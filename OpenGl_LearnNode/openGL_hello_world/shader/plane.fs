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

float BIAS =  0.001;
int SampleNum = 10;

float Bias()
{
    vec3 lightDir=normalize(lightPos-fs_in.FragPos);
    float bias = max(BIAS * (1.0 - dot(fs_in.Normal, lightDir)), BIAS/5.0);
    return bias;
}

float PCF(vec3 projectCoords)
{
    float currentDepth=projectCoords.z;
    float visible=0.0;   
    
    float bias = Bias();
    vec2 textlSize = 1.0 / textureSize(shadowMap, 0);
    for(int i=0;i<SampleNum;++i)
    {
        float closeDepth=texture(shadowMap,projectCoords.xy + textlSize * 9).r;
        visible += currentDepth < closeDepth + bias ? 1.0 : 0.0;
    }

    visible/=float(SampleNum);

    return visible;
}

float VisibleCalculation(vec3 projectCoords)
{
     float visible=0.0;   
     float currentDepth=projectCoords.z;

     if(currentDepth > 1.0)
            return 0.0;
    //硬阴影
    if(false)
    {     
        float closeDepth=texture(shadowMap,projectCoords.xy).r;
        
        //float bias=0.005;
        float bias = Bias();
        visible =currentDepth < closeDepth + bias? 1.0 : 0.0;
        return visible;
    }
    else
    {      
        float bias = Bias();
        vec2 textlSize = 1.0 / textureSize(shadowMap, 0);
        for (int x = -2; x <= 2; ++x)
        {
            for (int y = -2; y <= 2; ++y)
            {
                float closeDepth = texture(shadowMap, vec2(x, y) * textlSize + projectCoords.xy).r;
                visible += currentDepth < closeDepth + bias ? 1.0 : 0.0;
            }
        }
        visible /= 25.0;
        return visible;
    }
}

vec3 BlinnPhong()
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
         
    vec3 lighting = (ambient +   (diffuse + specular)) * color; 

    return lighting;
}

void main()
{   
    vec3 projectCoords=fs_in.FragPosLightSpace.xyz/fs_in.FragPosLightSpace.w;
    projectCoords = projectCoords * 0.5 + 0.5;          //xyz=>[-1,1]  ===> [0,1]
     // 计算阴影
    //float visible = VisibleCalculation(projectCoords);  
    float visible = PCF(projectCoords);  
    vec3 color = BlinnPhong();

    FragColor = vec4(visible * color , 1.0f);
}