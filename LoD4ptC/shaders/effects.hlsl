#pragma pack_matrix(row_major)

//--------------------------------------------------------------------------------------
// Constant buffers
//--------------------------------------------------------------------------------------

cbuffer perObject
{
    float4x4 g_wvp;
    float4x4 g_world; 
    float4 g_lightDir;
    float4 g_lightColor;
    float4 g_cameraPos; 
    float g_splatradius;
    float g_splatdiameter;
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
    float3 r = reflect(g_lightDir.xyz, normal);
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

//Ditches normals
PosCol VS_SIMPLE(float4 inPos : POSITION, float3 inNormal : NORMAL, float4 inColor : COLOR)
{
    PosCol output;

    inPos.w = 1.0f;

    output.pos = inPos;
    output.color = inColor;
    return output;
}

//--------------------------------------------------------------------------------------
// Geometry Shaders
//--------------------------------------------------------------------------------------

//only color for no lighting quads
[maxvertexcount(4)]
void GS_QUAD(point PosCol input[1], inout TriangleStream<PosCol> OutStream)
{
    input[0].pos = mul(input[0].pos, g_wvp);

    input[0].pos.xy += float2(-g_splatradius, +g_splatradius); //left up
    OutStream.Append(input[0]);

    input[0].pos.y -= g_splatdiameter; //right up
    OutStream.Append(input[0]);

    input[0].pos.xy += float2(g_splatdiameter, g_splatdiameter); //left down
    OutStream.Append(input[0]);

    input[0].pos.y -= g_splatdiameter; //right down
    OutStream.Append(input[0]);
	

    OutStream.RestartStrip();

}

//only color for no lighting quads
//Adds tex coords to render circle
[maxvertexcount(4)]
void GS_CIRCLE(point PosCol input[1], inout TriangleStream<PosColTex> OutStream)
{
    PosColTex output;
    output.pos = mul(input[0].pos, g_wvp);
    output.color = input[0].color;

    output.pos.xy += float2(-g_splatradius, +g_splatradius); //left up
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter; //right up
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos.xy += float2(g_splatdiameter, g_splatdiameter); //left down
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter; //right down
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);

    OutStream.RestartStrip();
}


//Lit quad splats
[maxvertexcount(4)]
void GS_QUAD_LIT(point PosNorCol input[1], inout TriangleStream<PosWorldNorCol> OutStream)
{
    PosWorldNorCol output;
    output.pos = mul(input[0].pos, g_wvp);
    output.posWorld = mul(input[0].pos, g_world);
    output.normal = mul(input[0].normal, (float3x3) g_world);
    output.color = input[0].color;

    output.pos.xy += float2(-g_splatradius, +g_splatradius); //left up
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter; //right up
    OutStream.Append(output);

    output.pos.xy += float2(g_splatdiameter, g_splatdiameter); //left down
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter; //right down
    OutStream.Append(output);

    OutStream.RestartStrip();

}

//lit circle splats
[maxvertexcount(4)]
void GS_CIRCLE_LIT(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, g_wvp);
    output.posWorld = mul(input[0].pos, g_world);
    output.normal = mul(input[0].normal, (float3x3) g_world);
    output.color = input[0].color;

    output.pos.xy += float2(-g_splatradius, +g_splatradius); //left up
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter; //right up
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos.xy += float2(g_splatdiameter, g_splatdiameter); //left down
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos.y -= g_splatdiameter; //right down
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);

    OutStream.RestartStrip();
}
//--------------------------------------------------------------------------------------
// Pixel Shaders
//--------------------------------------------------------------------------------------

float4 PS_QUAD_NOLIGHT(PosCol input) : SV_TARGET
{
    return input.color;
}

float4 PS_CIRCLE_NOLIGHT(PosColTex input) : SV_TARGET
{
    if (input.tex.x * input.tex.x + input.tex.y * input.tex.y > 1)
        discard;

    return input.color;
}


float4 PS_QUAD_PHONG(PosWorldNorCol input) : SV_TARGET
{
    return lightning_phong(input.posWorld, input.normal) * input.color;
//    return g_lightColor;
}

float4 PS_CIRCLE_PHONG(PosWorldNorColTex input) : SV_TARGET
{
    if (input.tex.x * input.tex.x + input.tex.y * input.tex.y > 1)
    {
        discard;            //remove discard when using blending for better performance 
        return float4(0, 0, 0, 0); 
    }

    return lightning_phong(input.posWorld, input.normal)*input.color;
}

