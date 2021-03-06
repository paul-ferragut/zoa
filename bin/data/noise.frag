#version 120
uniform float time;
uniform float fluidity1;
uniform float fluidity2;
uniform int fluidity3;

uniform float colorBlendingGradientX;
uniform float colorBlendingGradientY;

uniform float addBlurSurface1;
uniform float addBlurSurface2;

uniform float addInnerSurface1;
uniform float addInnerSurface2;

uniform float moveContrast;

uniform float fillThreshold;



uniform int width;
uniform int height;

uniform float scaleWidth;
uniform float scaleHeight;



//uniform sampler2D tex0;     // base image to renderRect
//uniform sampler2DRect gradient; // color gradient to applyRect
//uniform float gradientWidth;    // gradient width



vec4 mod289(vec4 x)
{
return x - floor(x * (1.0 / 289.0)) * 289.0;
}

vec4 permute(vec4 x)
{
return mod289(((x*34.0)+1.0)*x);
}

vec4 taylorInvSqrt(vec4 r)
{
return 1.79284291400159 - 0.85373472095314 * r;
}

vec2 fade(vec2 t) {
return t*t*t*(t*(t*6.0-15.0)+10.0);
}

// Classic Perlin noise
float cnoise(vec2 P)
{
vec4 Pi = floor(P.xyxy) + vec4(0.0, 0.0, 1.0, 1.0);
vec4 Pf = fract(P.xyxy) - vec4(0.0, 0.0, 1.0, 1.0);
Pi = mod289(Pi); // To avoid truncation effects in permutation
vec4 ix = Pi.xzxz;
vec4 iy = Pi.yyww;
vec4 fx = Pf.xzxz;
vec4 fy = Pf.yyww;

vec4 i = permute(permute(ix) + iy);

vec4 gx = fract(i * (1.0 / 41.0)) * 2.0 - 1.0 ;
vec4 gy = abs(gx) - 0.5 ;
vec4 tx = floor(gx + 0.5);
gx = gx - tx;

vec2 g00 = vec2(gx.x,gy.x);
vec2 g10 = vec2(gx.y,gy.y);
vec2 g01 = vec2(gx.z,gy.z);
vec2 g11 = vec2(gx.w,gy.w);

vec4 norm = taylorInvSqrt(vec4(dot(g00, g00), dot(g01, g01), dot(g10, g10), dot(g11, g11)));
g00 *= norm.x;
g01 *= norm.y;
g10 *= norm.z;
g11 *= norm.w;

float n00 = dot(g00, vec2(fx.x, fy.x));
float n10 = dot(g10, vec2(fx.y, fy.y));
float n01 = dot(g01, vec2(fx.z, fy.z));
float n11 = dot(g11, vec2(fx.w, fy.w));

vec2 fade_xy = fade(Pf.xy);
vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade_xy.x);
float n_xy = mix(n_x.x, n_x.y, fade_xy.y);
return 2.3 * n_xy;
}

// Classic Perlin noise, periodic variant
float pnoise(vec2 P, vec2 rep)
{
vec4 Pi = floor(P.xyxy) + vec4(0.0, 0.0, 1.0, 1.0);
vec4 Pf = fract(P.xyxy) - vec4(0.0, 0.0, 1.0, 1.0);
Pi = mod(Pi, rep.xyxy); // To create noise with explicit period
Pi = mod289(Pi); // To avoid truncation effects in permutation
vec4 ix = Pi.xzxz;
vec4 iy = Pi.yyww;
vec4 fx = Pf.xzxz;
vec4 fy = Pf.yyww;

vec4 i = permute(permute(ix) + iy);

vec4 gx = fract(i * (1.0 / 41.0)) * 2.0 - 1.0 ;
vec4 gy = abs(gx) - 0.5 ;
vec4 tx = floor(gx + 0.5);
gx = gx - tx;

vec2 g00 = vec2(gx.x,gy.x);
vec2 g10 = vec2(gx.y,gy.y);
vec2 g01 = vec2(gx.z,gy.z);
vec2 g11 = vec2(gx.w,gy.w);

vec4 norm = taylorInvSqrt(vec4(dot(g00, g00), dot(g01, g01), dot(g10, g10), dot(g11, g11)));
g00 *= norm.x;
g01 *= norm.y;
g10 *= norm.z;
g11 *= norm.w;

float n00 = dot(g00, vec2(fx.x, fy.x));
float n10 = dot(g10, vec2(fx.y, fy.y));
float n01 = dot(g01, vec2(fx.z, fy.z));
float n11 = dot(g11, vec2(fx.w, fy.w));

vec2 fade_xy = fade(Pf.xy);
vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade_xy.x);
float n_xy = mix(n_x.x, n_x.y, fade_xy.y);
return 2.3 * n_xy;
}

float fbm(vec2 P, int octaves, float lacunarity, float gain)
{
float sum = 0.0;
float amp = 1.0;
vec2 pp = P;
   

for(int i = 0; i < 40; i++)
{
    if(i<octaves){
        amp *= gain;
        sum += amp * cnoise(pp);
        pp *= lacunarity;
    }else return sum;

}
return sum;

}


float pattern(in vec2 p) {
float l = 2.5;
float g = 0.4;
int oc = 10;

vec2 q = vec2( fbm( p + vec2(0.0,0.0),oc,l,g),fbm( p + vec2(5.2,1.3),oc,l,g));
vec2 r = vec2( fbm( p + 4.0*q + vec2(1.7,9.2),oc,l,g ), fbm( p + 4.0*q + vec2(8.3,2.8) ,oc,l,g));
return fbm( p + 4.0*r ,oc,l,g);
}

float pattern2( in vec2 p, out vec2 q, out vec2 r , in float time)
{
//float l = 2.3;
//float g = 0.4;
//int oc = 10;
float l=fluidity1;
float g=fluidity2;
int oc=fluidity3;


q.x = fbm( p + vec2(time,time),oc,l,g);
q.y = fbm( p + vec2(5.2*time,1.3*time) ,oc,l,g);

r.x = fbm( p + 4.0*q + vec2(1.7,9.2),oc,l,g );
r.y = fbm( p + 4.0*q + vec2(8.3,2.8) ,oc,l,g);

return fbm( p + 4.0*r ,oc,l,g);
}




void main() {

vec2 q = gl_FragCoord.xy / vec2(scaleWidth,scaleHeight);
vec2 p = -1.0 + 2.0 * q;
vec2 qq;
vec2 r;
float color = pattern2(p,qq,r,time);

//vec4 c = vec4(color,color,color,color);
//vec4 c = vec4(color * p.x + qq.y + qq.x, qq.x + p.y * color + r.y + r.y, qq.x * r.x + qq.x+color,1.);
//vec4 c = vec4(color * red1 + green2 + green1, green1 + red2 * color + red3 + red3, green1 *  blue1 +  green1+color,1.);
//vec4 c = vec4(  color *red1+red2+red3,  green3+green4* color +green1+green2,blue3* blue2 + blue1+color,1.);
//vec4 c = vec4(color * (p.x*colorBlendingGradientX) + (qq.y*addBlurSurface1) + (qq.x*addBlurSurface1), (qq.x*moveContrast) + (p.y*colorBlendingGradientY) * color + (r.y*addInnerSurface1) + (r.y*addInnerSurface2) , qq.x *  r.x + (qq.x*fillThreshold)+color,1.);
    
    //vec4 c = vec4(,,,1.);
    
   // color * (p.x*colorBlendingGradientX) + (qq.y*addBlurSurface1) + (qq.x*addBlurSurface1)
   // (qq.x*moveContrast) + (p.y*colorBlendingGradientY) * color + (r.y*addInnerSurface1) + (r.y*addInnerSurface2)
   // qq.x *  r.x + (qq.x*fillThreshold)+color
    
    // color * (p.x*colorBlendingGradientX) + (qq.y*addBlurSurface1) + (qq.x*addBlurSurface1)
    // (qq.x*moveContrast) + (p.y*colorBlendingGradientY) * color + (r.y*addInnerSurface1) + (r.y*addInnerSurface2)
    // qq.x *  r.x + (qq.x*fillThreshold)+color
    
   // vec4 c = vec4(qq.x *  r.x + (qq.x*fillThreshold)+color, qq.x *  r.x + (qq.x*fillThreshold)+color, qq.x *  r.x + (qq.x*fillThreshold)+color,1.);
 vec4 c = vec4(color,color, color,1.);

c *= 3.5;

    
// float gray = 0.2989 * c.r + 0.5870 * c.g + 0.1140 * c.b;
    
     float gray = c.r + c.g + c.b;
    
   // vec2 pos = gl_TexCoord[0].st;
   // vec4 src = texture2DRect(tex0, pos);

   // vec4 map = texture2DRect(gradient, vec2(gray* gradientWidth*0.3,gray* gradientWidth*0.3) );//Rect
    



    gl_FragColor = vec4( 1.0,1.0,1.0,gray );
    
//gl_FragColor = c;
}
