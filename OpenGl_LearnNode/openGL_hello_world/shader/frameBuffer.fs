#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D screenTexture;

//得到反相
vec3 reversedPhase()
{
   return vec3(1.0 - texture(screenTexture, TexCoords));
}

//灰度化
float getGray()
{
    FragColor = texture(screenTexture, TexCoords);
    float average = (FragColor.r + FragColor.g + FragColor.b) / 3.0;
    return average;
}

//锐化->由于纹理进行二次操作，所以可以得到相邻的顶点值，所以可以经常一系列操作
vec3 gradation()
{
    const float offset = 1.0 / 300.0;

    //获取当前点相邻的点值
    vec2 offsets[9] = vec2[](
        vec2(-offset,  offset), // 左上
        vec2( 0.0f,    offset), // 正上
        vec2( offset,  offset), // 右上
        vec2(-offset,  0.0f),   // 左
        vec2( 0.0f,    0.0f),   // 中
        vec2( offset,  0.0f),   // 右
        vec2(-offset, -offset), // 左下
        vec2( 0.0f,   -offset), // 正下
        vec2( offset, -offset)  // 右下
    );

    //锐化卷积核
    //float kernel[9] = float[](
    //    -1, -1, -1,
    //    -1,  9, -1,
    //    -1, -1, -1
    //);
    

    //模糊卷积核
    //float kernel[9] = float[](
    //1.0 / 16, 2.0 / 16, 1.0 / 16,
    //2.0 / 16, 4.0 / 16, 2.0 / 16,
    //1.0 / 16, 2.0 / 16, 1.0 / 16  
    //);

    //边缘检测卷积核
    float kernel[9] = float[](
        1, 1, 1,
        1, -8, 1,
        1, 1, 1
    );

    //对相邻点(3x3)进行采样
    vec3 sampleTex[9];
    for(int i = 0; i < 9; i++)
    {
        sampleTex[i] = vec3(texture(screenTexture, TexCoords.st + offsets[i]));
    }
    vec3 col = vec3(0.0);
    //得到卷积后的结果
    for(int i = 0; i < 9; i++)
        col += sampleTex[i] * kernel[i];

    return col;
}



void main()
{ 
    ///深度贴图只有r（一个）数据
    vec3 col = vec3(texture(screenTexture, TexCoords).r);
    FragColor = vec4(col,1.0);
    //FragColor = vec4(vec3(1.0 - texture(screenTexture, TexCoords)), 1.0);
    //FragColor = vec4(vec3(getGray()), 1.0);

    //FragColor = vec4(gradation(), 1.0);
}