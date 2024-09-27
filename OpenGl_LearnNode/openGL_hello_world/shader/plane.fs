#version 330 core
out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec4 FragPosLightSpace;
} fs_in;

uniform sampler2D diffuseTexture;
uniform sampler2D shadowMap;

uniform vec3 lightPos;
uniform vec3 viewPos;

float ShadowCalculation(vec4 fragPosLightSpace)
{
     vec3 projectCoords=fragPosLightSpace.xyz/fragPosLightSpace.w;
     projectCoords = projectCoords * 0.5 + 0.5;          //xyz=>[-1,1]  ===> [0,1]
     float currentDepth=projectCoords.z;

     if(currentDepth > 1.0)
            return 0.0;
    //”≤“ı”∞
    if(false)
    {     
        float closeDepth=texture(shadowMap,projectCoords.xy).r;
        

        vec3 lightDir=normalize(lightPos-fs_in.FragPos);
        //float bias=0.005;
        float bias = max(0.05 * (1.0 - dot(fs_in.Normal, lightDir)), 0.005);
        float shadow=currentDepth > closeDepth + bias? 1.0 : 0.0;
        return shadow;
    }
    else
    {
        float shadow=0.0;

        vec3 lightDir = normalize(lightPos - fs_in.FragPos);
        float bias = max(0.05 * (1.0 - dot(fs_in.Normal, lightDir)), 0.005);

        vec2 textlSize = 1.0 / textureSize(shadowMap, 0);
        for (int x = -2; x <= 2; ++x)
        {
            for (int y = -2; y <= 2; ++y)
            {
                float closeDepth = texture(shadowMap, vec2(x, y) * textlSize + projectCoords.xy).r;
                shadow += currentDepth > closeDepth + bias ? 1.0 : 0.0;
            }
        }
        shadow /= 25.0;
        return shadow;
    }
}

void main()
{           
    vec3 color = texture(diffuseTexture, fs_in.TexCoords).rgb;
    vec3 normal = normalize(fs_in.Normal);
    vec3 lightColor = vec3(1.0);
    // Ambient
    vec3 ambient = 0.15 * color;
    // Diffuse
    vec3 lightDir = normalize(lightPos - fs_in.FragPos);
    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * lightColor;
    // Specular
    vec3 viewDir = normalize(viewPos - fs_in.FragPos);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = 0.0;
    vec3 halfwayDir = normalize(lightDir + viewDir);  
    spec = pow(max(dot(normal, halfwayDir), 0.0), 64.0);
    vec3 specular = spec * lightColor;    
    // º∆À„“ı”∞
    float shadow = ShadowCalculation(fs_in.FragPosLightSpace);       
    vec3 lighting = (ambient + (1.0 - shadow) * (diffuse + specular)) * color;    

    FragColor = vec4(lighting, 1.0f);
}