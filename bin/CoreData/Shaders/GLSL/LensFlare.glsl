#include "Uniforms.glsl"
#include "Samplers.glsl"
#include "Transform.glsl"
#include "ScreenPos.glsl"
#include "Lighting.glsl"
#include "PostProcess.glsl"


varying vec2 vTexCoord;
varying vec2 vScreenPos;

uniform sampler2D sViewport0;

#ifdef LENSFLARE
uniform sampler2D sLensColorTex1;
#endif

#ifdef COMBINE
uniform sampler2D sLensFlareTex1;
uniform float cMixRatio;
#endif


#ifdef LENSFLARE
uniform vec3 cLensFlareScale = vec3(9.431);   // 9.431
uniform vec3 cLensFlareBias  = vec3(-0.493);   // -0.493

uniform int cLensFlareGhosts  = 2;
uniform float cLensFlareDispersal = 0.25;   // 0.25

uniform float cLensFlareHaloWidth = 0.3;    // 0.5
uniform float cLensFlareDistortion = 1.0;   // -0.914
#endif

void VS()
{
    mat4 modelMatrix = iModelMatrix;
    vec3 worldPos = GetWorldPos(modelMatrix);
    gl_Position = GetClipPos(worldPos);
    vTexCoord = GetQuadTexCoord(gl_Position);
    vScreenPos = GetScreenPosPreDiv(gl_Position);
    
   

    
    
    // flipping the texture coordinates
//    vScreenPos = -vScreenPos + vec2(1.0);
    
    
    #ifdef PRIOR
        //vScreenPos = vScreenPos * 0.5 + 0.5;
       // gl_Position = vec4(vScreenPos, 0.0, 1.0);
    vScreenPos = vScreenPos * vec2(5) + vec2(5);
    #endif
    
    
    
}


vec4 textureDistorted(in sampler2D tex,
                      in vec2 texcoord,
                      in vec2 direction,
                      in vec3 distortion
                      ) {
    vec4 result = vec4(
                texture(tex, texcoord + direction * distortion.r).r,
                texture(tex, texcoord + direction * distortion.g).g,
                texture(tex, texcoord + direction * distortion.b).b,
                1.0
                );
    
    return result;
}



void PS()
{

#ifdef PRIOR
    vec4 tScale = vec4(10.0);
    vec4 tBias = vec4(-0.8);

    gl_FragColor = max(vec4(0.0), texture2D(sViewport0, vScreenPos) + tBias) * tScale;
#endif
    
    
    
#ifdef LENSFLARE
    //vec4 original = texture2D(sViewport0, vScreenPos);
    
    vec2 texcoordFlipped = -vScreenPos + vec2(1.0);

 //   vec4 result = texture2D(sViewport0, texcoordFlipped);
 //   result = vec4((max(vec3(0.0), result.rgb + -0.8) * 5), result.a);
    
    vec2 texelSize = 1.0 / vec2(textureSize(sViewport0, 0));
    
    vec3 distortion = vec3(-texelSize.x * cLensFlareDistortion, 0.0, texelSize.x * cLensFlareDistortion);
    
    
    
    
    // ghost vector to image centre
    vec2 ghostVec = (vec2(0.5) - texcoordFlipped) * cLensFlareDispersal;

    int uGhosts = 4;
    // sample ghosts
    vec4 result = vec4(0.0);
    for (int i = 0; i < uGhosts; ++i) {
        vec2 offset = fract(texcoordFlipped + ghostVec * float(i));
        
        float weight = length(vec2(0.5) - offset) / length(vec2(0.5));
        weight = pow(1.0 - weight, 10.0);
        
      //  vec4 test = texture2D(sViewport0, offset) ;
      //  test = vec4((max(vec3(0.0), test.rgb + cLensFlareBias ) * cLensFlareScale), 1.0);
        
       // result += weight;
        result += textureDistorted(sViewport0,
                                  offset,
                                   normalize(ghostVec),
                                   distortion
                                   ) * weight;
    }

    // lensColor

    vec4 lenscolor = texture2D(sLensColorTex1, vec2(length(vec2(0.5) - texcoordFlipped) / length(vec2(0.5))) );
    result *= lenscolor;
    
    
    // sample halo
    vec2 haloVec = normalize(ghostVec) * cLensFlareHaloWidth;
    
    float weight = length(vec2(0.5) - fract(texcoordFlipped + haloVec)) / length(vec2(0.5));
    weight = pow(1.0 - weight, 10.0);
    vec4 halo = textureDistorted(
                     sViewport0,
                     fract(texcoordFlipped + haloVec),
                     normalize(ghostVec),
                     distortion
                     ) * weight;
   //	halo = texture(sViewport0, texcoordFlipped + haloVec) * weight;
    
    result += halo;
    
    
    //result = GaussianBlur(3, vec2(1.0 0.0), 1.0, 2.0, result, texcoordFlipped);
    //vec4 GaussianBlur(int blurKernelSize, vec2 blurDir, vec2 blurRadius, float sigma, sampler2D texSampler, vec2 texCoord)

    // blur
    

     //vec4 result2 = incrementalGauss1D(sViewport0, texelSize, vScreenPos, 5, vec2(1.0, 0.0));
    
    
    // Prevent oversaturation
    //original *= max(vec4(1.0) - result, vec4(0.0,0.0,0.0,1.0));
    

    gl_FragColor = result;
    
#endif
    
#ifdef COMBINE
    vec4 original = texture2D(sViewport0, vScreenPos);
    vec4 effect = texture2D(sLensFlareTex1, vTexCoord);
    gl_FragColor = mix(original, effect, 0.9);
    //gl_FragColor = effect;
#endif

}

