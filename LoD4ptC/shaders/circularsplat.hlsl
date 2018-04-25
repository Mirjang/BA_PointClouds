#pragma pack_matrix(row_major)

cbuffer perObject
{
	float4x4 m_wvp;
    float4 lightpos; 
    float splatradius;
	float splatdiameter; 
};


struct VS_OUTPUT
{
	float4 pos : SV_POSITION;
	float3 normal : NORMAL;
    float4 color : COLOR; 
};

struct PS_INPUT
{
    float4 pos : SV_POSITION;
	float2 tex : TEXCOORD;	//UV coords to determine if pixel is in circle
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
void GS(point VS_OUTPUT input[1], inout TriangleStream<PS_INPUT> OutStream)
{
	PS_INPUT output;
	output.pos = mul(input[0].pos, m_wvp);
    output.normal = input[0].normal;
    output.color = input[0].color;

	output.pos.xy += float2(-splatradius, +splatradius);//left up
	output.tex.xy = float2(-1, -1); 
	OutStream.Append(output);

	output.pos.y -= splatdiameter;	//right up
	output.tex.xy = float2(1, -1);
	OutStream.Append(output);

	output.pos.xy += float2(splatdiameter, splatdiameter); //left down
	output.tex.xy = float2(-1, 1);
	OutStream.Append(output);

	output.pos.y -= splatdiameter; //right down
	output.tex.xy = float2(1, 1);
	OutStream.Append(output);


	OutStream.RestartStrip();
}


float4 PS(PS_INPUT input) : SV_TARGET
{
	if (input.tex.x*input.tex.x + input.tex.y * input.tex.y > 1)
		discard; 

    return input.color; 
}
