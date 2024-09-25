#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D screenTexture;

//�õ�����
vec3 reversedPhase()
{
   return vec3(1.0 - texture(screenTexture, TexCoords));
}

//�ҶȻ�
float getGray()
{
    FragColor = texture(screenTexture, TexCoords);
    float average = (FragColor.r + FragColor.g + FragColor.b) / 3.0;
    return average;
}

//��->����������ж��β��������Կ��Եõ����ڵĶ���ֵ�����Կ��Ծ���һϵ�в���
vec3 gradation()
{
    const float offset = 1.0 / 300.0;

    //��ȡ��ǰ�����ڵĵ�ֵ
    vec2 offsets[9] = vec2[](
        vec2(-offset,  offset), // ����
        vec2( 0.0f,    offset), // ����
        vec2( offset,  offset), // ����
        vec2(-offset,  0.0f),   // ��
        vec2( 0.0f,    0.0f),   // ��
        vec2( offset,  0.0f),   // ��
        vec2(-offset, -offset), // ����
        vec2( 0.0f,   -offset), // ����
        vec2( offset, -offset)  // ����
    );

    //�񻯾����
    //float kernel[9] = float[](
    //    -1, -1, -1,
    //    -1,  9, -1,
    //    -1, -1, -1
    //);
    

    //ģ�������
    //float kernel[9] = float[](
    //1.0 / 16, 2.0 / 16, 1.0 / 16,
    //2.0 / 16, 4.0 / 16, 2.0 / 16,
    //1.0 / 16, 2.0 / 16, 1.0 / 16  
    //);

    //��Ե�������
    float kernel[9] = float[](
        1, 1, 1,
        1, -8, 1,
        1, 1, 1
    );

    //�����ڵ�(3x3)���в���
    vec3 sampleTex[9];
    for(int i = 0; i < 9; i++)
    {
        sampleTex[i] = vec3(texture(screenTexture, TexCoords.st + offsets[i]));
    }
    vec3 col = vec3(0.0);
    //�õ������Ľ��
    for(int i = 0; i < 9; i++)
        col += sampleTex[i] * kernel[i];

    return col;
}



void main()
{ 
    ///�����ͼֻ��r��һ��������
    vec3 col = vec3(texture(screenTexture, TexCoords).r);
    FragColor = vec4(col,1.0);
    //FragColor = vec4(vec3(1.0 - texture(screenTexture, TexCoords)), 1.0);
    //FragColor = vec4(vec3(getGray()), 1.0);

    //FragColor = vec4(gradation(), 1.0);
}