#pragma pack_matrix(row_major)

//--------------------------------------------------------------------------------------
// Constant buffers
//--------------------------------------------------------------------------------------

cbuffer perObject
{
	float4x4 g_wvp;
    float4 g_lightpos;
    float4 g_lightColor;
    float g_splatradius;
	float g_splatdiameter; 
};

//--------------------------------------------------------------------------------------
// Structs
//--------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------
// Rasterizer states
//--------------------------------------------------------------------------------------

RasterizerState rsDefault
{
};

RasterizerState rsCullFront
{
    CullMode = Front;
};

RasterizerState rsCullBack
{
    CullMode = Back;
};

RasterizerState rsCullNone
{
    CullMode = None;
};

RasterizerState rsLineAA
{
    CullMode = None;
    AntialiasedLineEnable = true;
};

//--------------------------------------------------------------------------------------
// DepthStates
//--------------------------------------------------------------------------------------
DepthStencilState EnableDepth
{
    DepthEnable = TRUE;
    DepthWriteMask = ALL;
    DepthFunc = LESS_EQUAL;
};

BlendState NoBlending
{
    AlphaToCoverageEnable = FALSE;
    BlendEnable[0] = FALSE;
};

//--------------------------------------------------------------------------------------
// Shaders
//--------------------------------------------------------------------------------------

PosNorCol VS_PASSTHROUGH(float4 inPos : POSITION, float3 inNormal : NORMAL, float4 inColor : COLOR)
{
    PosNorCol output;

	inPos.w = 1.0f;

    output.pos = inPos;
    output.normal = inNormal;
    
    output.color = inColor; 
    

	return output;
}

[maxvertexcount(4)]
void GS_QUAD(point PosNorCol input[1], inout TriangleStream<PosNorCol> OutStream)
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

//Adds tex coords to render circle
[maxvertexcount(4)]
void GS_TEXCOORDS(point PosNorCol input[1], inout TriangleStream<PosNorColTex> OutStream)
{
    PosNorColTex output;
    output.pos = mul(input[0].pos, g_wvp);
    output.normal = input[0].normal;
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



float4 PS_QUAD_NOLIGHT(PosNorCol input) : SV_TARGET
{
    return input.color;
}

float4 PS_CIRCLE_NOLIGHT(PosNorColTex input) : SV_TARGET
{
    if (input.tex.x * input.tex.x + input.tex.y * input.tex.y > 1)
        discard;

    return input.color;
}


//--------------------------------------------------------------------------------------
// Techniques
//--------------------------------------------------------------------------------------
technique11 quad_noLight
{
    pass P0
    {
        SetVertexShader(CompileShader(vs_4_0, VS_PASSTHROUGH()));
        SetGeometryShader(CompileShader(gs_4_0, GS_QUAD()));
        SetPixelShader(CompileShader(ps_4_0, PS_QUAD_NOLIGHT()));
        
        SetRasterizerState(rsCullNone);
        SetDepthStencilState(EnableDepth, 0);
        SetBlendState(NoBlending, float4(0.0f, 0.0f, 0.0f, 0.0f), 0xFFFFFFFF);
    }
}


technique11 circle_noLight
{
    pass P0
    {
        SetVertexShader(CompileShader(vs_4_0, VS_PASSTHROUGH()));
        SetGeometryShader(CompileShader(gs_4_0, GS_TEXCOORDS()));
        SetPixelShader(CompileShader(ps_4_0, PS_CIRCLE_NOLIGHT()));
        
        SetRasterizerState(rsCullNone);
        SetDepthStencilState(EnableDepth, 0);
        SetBlendState(NoBlending, float4(0.0f, 0.0f, 0.0f, 0.0f), 0xFFFFFFFF);
    }
}