#pragma pack_matrix(row_major)


//--------------------------------------------------------------------------------------
// Defines
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// Constant buffers
//--------------------------------------------------------------------------------------


cbuffer splatSizeBuffer : register(b0)
{
    float2 g_splatradius;
    float2 g_splatdiameter;
    uint g_LODdepth;
};


cbuffer perObject : register(b1)
{
    float4x4 g_wvp;
    float4x4 g_world; 
    float4 g_lightDir;
    float4 g_lightColor;
    float4 g_cameraPos; 
    float4 g_cameraDir; 
};


cbuffer shaderSettings : register(b2)
{
    float2 g_splatSize;
    float g_pixelThreshhold;
    float g_screenHeightDiv2; 
    float g_aspectRatio;
    uint g_maxLod; 
};



//--------------------------------------------------------------------------------------
// Structs
//--------------------------------------------------------------------------------------

struct PosCol
{
    float4 pos : SV_POSITION;
    float4 color : COLOR;
};

struct PosColTex
{
    float4 pos : SV_POSITION;
    float4 color : COLOR;
    float2 tex : TEXCOORD; //UV coords to determine if pixel is in circle
};

struct PosNorCol
{
    float4 pos : SV_POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
};

struct PosNorColTex
{
    float4 pos : SV_POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
    float2 tex : TEXCOORD; //UV coords to determine if pixel is in circle
};

struct PosWorldNorCol
{
    float4 pos : SV_POSITION;
    float3 posWorld : POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
};

struct PosWorldNorColTex
{
    float4 pos : SV_POSITION;
    float3 posWorld : POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
    float2 tex : TEXCOORD; //UV coords to determine if pixel is in circle
};

//--------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------

inline float4 lightning_phong(float3 worldPos, float3 normal)
{
    float3 r = reflect(-g_lightDir.xyz, normal);
    float3 v = normalize(g_cameraPos.xyz - worldPos);
    
    //lighting constants
    float cdiff = 0.5f;
    float cspec = 0.4f;
    float espec = 200; // specular exponent
    float camb = 0.15f;
    float cglow = 0.0f;


	//return g_Diffuse.Sample(samAnisotropic, Input.Tex);
    return (cdiff * saturate(dot(normal, g_lightDir.xyz))
		+ cspec * pow(saturate(dot(r, v)), espec)
		+ cglow) * g_lightColor;
};


//--------------------------------------------------------------------------------------
// Vertex Shaders
//--------------------------------------------------------------------------------------

//magic happens in GS
PosNorCol VS_PASSTHROUGH(float4 inPos : POSITION, float3 inNormal : NORMAL, float4 inColor : COLOR)
{
    PosNorCol output;

    inPos.w = 1.0f;

    output.pos = inPos;
    output.normal = inNormal;
    
    output.color = inColor;
    

    return output;
}

//sets color based on LOD
PosNorCol VS_APPLY_DEPTHCOLOR(float4 inPos : POSITION, float3 inNormal : NORMAL, float4 inColor : COLOR)
{
    PosNorCol output;

    inPos.w = 1.0f;

    output.pos = inPos;
    output.normal = inNormal;
    
    float tmp = (1.0f * g_LODdepth) / g_maxLod; 

    output.color = float4(tmp, 1.0f - tmp, 0.0f, 1.0f);
    
    return output;
}


//--------------------------------------------------------------------------------------
// Geometry Shaders
//--------------------------------------------------------------------------------------

//only color for no lighting 
[maxvertexcount(4)]
void GS_UNLIT(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, g_wvp);
    output.normal = float3(0, 0, 0); 
    output.posWorld = float3(0, 0, 0); 
    output.color = input[0].color;


    output.pos.xy += float2(-1, 1)*g_splatradius; //left up
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter.y; //right up
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos.xy += g_splatdiameter; //left down
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter.y; //right down
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);

}


//lit splats
[maxvertexcount(4)]
void GS_LIT(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, g_wvp);
    output.posWorld = mul(input[0].pos, g_world);
    output.normal = mul(input[0].normal, (float3x3) g_world);
    output.color = input[0].color;

    output.pos.xy += float2(-1, 1) * g_splatradius; //left up
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter.y; //right up
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos.xy += g_splatdiameter; //left down
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter.y; //right down
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);
}

//only color for no lighting 
[maxvertexcount(4)]
void GS_UNLIT_ADAPTIVESPLATSIZE(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, g_wvp);
    output.normal = float3(0, 0, 0);
    output.posWorld = float3(0, 0, 0);
    output.color = input[0].color;

    float depth = output.pos.z / output.pos.w;

    float2 adaptedRadius = g_splatSize;
    
    for (int i = g_maxLod; i; --i)
    {
        if (g_pixelThreshhold < (g_splatSize.x * depth * 1080 / 2))
        {
            float scale = 1 << (g_maxLod - i);
            adaptedRadius = g_splatSize * float2(scale, scale);
        }
    }


    float2 adaptedDiameter = adaptedRadius + adaptedRadius;

    output.pos.xy += float2(-1, 1) * adaptedRadius; //left up
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos.y -= adaptedDiameter.y; //right up
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos.xy += adaptedDiameter; //left down
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos.y -= adaptedDiameter.y; //right down
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);

}

//--------------------------------------------------------------------------------------
// Pixel Shaders
//--------------------------------------------------------------------------------------

float4 PS_QUAD_NOLIGHT(PosWorldNorColTex input) : SV_TARGET
{
    return input.color;
}

float4 PS_CIRCLE_NOLIGHT(PosWorldNorColTex input) : SV_TARGET
{
    if (input.tex.x * input.tex.x + input.tex.y * input.tex.y > 1)
        discard;

    return input.color;
}


float4 PS_QUAD_PHONG(PosWorldNorColTex input) : SV_TARGET
{
    return lightning_phong(input.posWorld, input.normal) * input.color;
}

float4 PS_CIRCLE_PHONG(PosWorldNorColTex input) : SV_TARGET
{
    
    if (input.tex.x * input.tex.x + input.tex.y * input.tex.y > 1)
    {
        discard;            //remove discard when using blending for better performance 
        return float4(0, 0, 0, 0); 
    }

    return lightning_phong(input.posWorld, input.normal) * input.color;
}

// STASH

/*


//Ditches normals
PosNorCol VS_SIMPLE(float4 inPos : POSITION, float3 inNormal : NORMAL, float4 inColor : COLOR)
{
    PosNorCol output;

    inPos.w = 1.0f;

    output.pos = inPos;
    output.normal = inNormal;
    
    output.color = inColor;
    

    return output;
}

*/