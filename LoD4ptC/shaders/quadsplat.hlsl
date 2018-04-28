#pragma pack_matrix(row_major)

cbuffer perObject
{
	float4x4 g_wvp;
    float3 g_lightpos;
    float3 g_lightColor;
    float g_splatradius;
	float g_splatdiameter; 
};


struct VS_OUTPUT
{
	float4 pos : SV_POSITION;
	float3 normal : NORMAL;
    float4 color : COLOR; 
};


//Pass through
VS_OUTPUT VS(float4 inPos : POSITION, float3 inNormal : NORMAL, float4 inColor : COLOR)
{
	VS_OUTPUT output;

	inPos.w = 1.0f;

    output.pos = inPos;
    output.normal = inNormal;
    
    output.color = inColor; 
    
	return output;
}

[maxvertexcount(4)]
void GS(point VS_OUTPUT input[1], inout TriangleStream<VS_OUTPUT> OutStream)
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


float4 PS(VS_OUTPUT input) : SV_TARGET
{
    return input.color; 
}

