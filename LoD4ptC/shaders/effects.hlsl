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
    float4x4 m_world;
    float4x4 m_wvp;
    float4x4 m_view;
    float4 g_lightDir;
    float4 g_lightColor;
    float4 g_cameraPos; 
    float4 g_cameraDir; 
};


cbuffer shaderSettings : register(b2)
{
	float4 g_octreeMin; 
	float4 g_octreeRange; 
    float2 g_splatSize;
    float g_pixelThreshhold;
    float g_screenHeightDiv2; 
    float g_aspectRatio;
    uint g_maxLod; 
};

Texture1D<uint2> treeStructure : register(t0); 


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


struct PosNorColRad
{
    float4 pos : SV_POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
    float radius : PSIZE;  //PSIZE doesnt work? qq
};

struct PosNorColEllipticalAxis
{
    float4 pos : SV_POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
    float3 major : TANGENT0;
    float3 minor : TANGENT1;
};

struct PosWorldNorColEllipticalAxisTex
{
    float4 pos : SV_POSITION;
    float3 posWorld : POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
    float3 major : TANGENT0;
    float3 minor : TANGENT1; 
    float2 tex : TEXCOORD; //UV coords to determine if pixel is in circle
};


struct PosWorldNorColTexRadXY
{
    float4 pos : SV_POSITION;
    float3 posWorld : POSITION;
    float3 normal : NORMAL;
    float4 color : COLOR;
    float2 tex : TEXCOORD0; //UV coords to determine if pixel is in circle
    float2 radXY : TEXCOORD1; 
};

//--------------------------------------------------------------------------------------
// Functions
//--------------------------------------------------------------------------------------

inline uint calcDepth(float3 inpos)
{

    uint depth = 0; 

    uint2 node = treeStructure.Load(int2(0, 0)); //params:(index,unused) returns: x: child bits, y: first child offset
    uint nextIndex = 0;

    float3 center = g_octreeMin.xyz + g_octreeRange.xyz / 2;

    while (node.x)
    {
        uint offset = 0;


        uint depthShift = (4 << depth); 
        //determine subgrid of current pt
        float3 childRange = g_octreeRange.xyz / float3(depthShift, depthShift, depthShift);

        int3 distCheck = int3(center <= inpos.xyz);

        float3 signvec = 2 * distCheck - int3(1, 1, 1); 

        center += childRange * signvec;

        distCheck *= int3(1, 2, 4); 

        offset = distCheck.x + distCheck.y + distCheck.z; 

        uint childNr = 0;

        if (!(node.x & (1 << offset)))
            return depth;


        for (int i = 0; i < offset; ++i)
        {
            if (node.x & (1 << i)) //skip children that come before the cell the pt lies in
                ++childNr;
        }

        ++depth;
        nextIndex += node.y + childNr;
        node = treeStructure.Load(int2(nextIndex, 0));

        if(node.x&0xff00)   //leaf node
        {
            return g_maxLod; 
        }

    }
    return depth; 
}

inline float2 calcSplatSize(uint depth)
{
    
    float scale = (1 << g_maxLod - depth);
    return g_splatSize * float2(scale, scale);
}

inline float4 lightning_phong(float3 worldPos, float3 normal)
{
    float3 r = reflect(-g_lightDir.xyz, normal);
    float3 v = normalize(g_cameraPos.xyz - worldPos);
    
    //lighting constants
    float cdiff = 0.5f;
    float cspec = 0.4f;
    float espec = 200; // specular exponent
    float camb = 0.15f;


	//return g_Diffuse.Sample(samAnisotropic, Input.Tex);
    return (cdiff * saturate(dot(normal, g_lightDir.xyz))
		+ cspec * pow(saturate(dot(r, v)), espec)
		+ camb) * g_lightColor;
};


//--------------------------------------------------------------------------------------
// Vertex Shaders
//--------------------------------------------------------------------------------------

//magic happens in GS
PosNorCol VS_PASSTHROUGH(PosNorCol input)
{
    input.pos.w = 1.0f;
    return input;
}

//sets color based on LOD
PosNorCol VS_APPLY_DEPTHCOLOR(PosNorCol input)
{
    input.pos.w = 1.0f;     
    float tmp = (1.0f * g_LODdepth) / g_maxLod; 
    input.color = float4(tmp, 1.0f - tmp, 0.0f, 1.0f);  

    return input;
}

//magic happens in GS
PosNorColRad VS_RADIUS_PASSTHROUGH(PosNorColRad input)
{
    input.pos.w = 1.0f;
    return input;
}

//sets color based on LOD
PosNorColRad VS_RADIUS_APPLY_DEPTHCOLOR(PosNorColRad input)
{
    input.pos.w = 1.0f;
    float tmp = (1.0f * g_LODdepth) / g_maxLod;
    input.color = float4(tmp, 1.0f - tmp, 0.0f, 1.0f);

    return input;
}


PosNorColEllipticalAxis VS_ELLIPTICAL_PASSTHROUGH(PosNorColEllipticalAxis input)
{
    input.pos.w = 1.0f; 
    return input; 
}

PosNorColEllipticalAxis VS_ELLIPTICAL_APPLY_DEPTHCOLOR(PosNorColEllipticalAxis input)
{
    input.pos.w = 1.0f;
    float tmp = (1.0f * g_LODdepth) / g_maxLod;
    input.color = float4(tmp, 1.0f - tmp, 0.0f, 1.0f);
    return input;
}

//--------------------------------------------------------------------------------------
// Geometry Shaders
//--------------------------------------------------------------------------------------

//only color for no lighting 
[maxvertexcount(4)]
void GS_UNLIT(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, m_wvp);
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
    output.pos = mul(input[0].pos, m_wvp);
    output.posWorld = mul(input[0].pos, m_world);
    output.normal = mul(input[0].normal, (float3x3) m_world);
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
void GS_ADAPTIVESPLATSIZE(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, m_wvp);
    output.posWorld = mul(input[0].pos, m_world);
    output.normal = mul(input[0].normal, (float3x3) m_world);
    output.color = input[0].color;
    float depth = calcDepth(input[0].pos.xyz);
    
    float2 adaptedRadius = calcSplatSize(depth);
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

//only color for no lighting 
[maxvertexcount(4)]
void GS_ADAPTIVESPLATSIZE_DEPTHCOLOR(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, m_wvp);
    output.posWorld = mul(input[0].pos, m_world);
    output.normal = mul(input[0].normal, (float3x3) m_world);
    output.color = input[0].color;

    float depth = calcDepth(input[0].pos.xyz);
    
    float2 adaptedRadius = calcSplatSize(depth);
    float2 adaptedDiameter = adaptedRadius + adaptedRadius;

    float tmp = (1.0f * depth) / g_maxLod;

    output.color = float4(tmp, 1.0f - tmp, 0.0f, 1.0f);

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

//only color for no lighting 
[maxvertexcount(4)]
void GS_ORIENTED_ADAPTIVESPLATSIZE(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, m_wvp);
    output.posWorld = mul(input[0].pos, m_world);
    output.normal = mul(input[0].normal, (float3x3) m_world);
    output.color = input[0].color;
    float depth = calcDepth(input[0].pos.xyz);
    
    float2 adaptedRadius = calcSplatSize(depth);
  //  float2 adaptedDiameter = adaptedRadius + adaptedRadius;

    float4 axis1, axis2;
    axis1.xyz = mul(adaptedRadius.x, normalize(cross(input[0].normal, float3(0, 0, 1))));
    axis2.xyz = mul(adaptedRadius.x, normalize(cross(input[0].normal, axis1.xyz)));
    axis1.w = 0;
    axis2.w = 0;

    float4 pos = input[0].pos;
   
    output.pos = mul(pos - axis1, m_wvp);
//    output.color = float4(1, 0, 0, 1); 
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos = mul(pos - axis2, m_wvp);
//    output.color = float4(1, 1, 0, 1);
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos = mul(pos + axis2, m_wvp);
 //   output.color = float4(0, 1, 0, 1);
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos = mul(pos + axis1, m_wvp);
 //   output.color = float4(0, 0, 1, 1);
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);

}

//only color for no lighting 
[maxvertexcount(4)]
void GS_ORIENTED_ADAPTIVESPLATSIZE_DEPTHCOLOR(point PosNorCol input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, m_wvp);
    output.posWorld = mul(input[0].pos, m_world);
    output.normal = mul(input[0].normal, (float3x3) m_world);
    output.color = input[0].color;

    float depth = calcDepth(input[0].pos.xyz);
    

    float tmp = (1.0f * depth) / g_maxLod;
    output.color = float4(tmp, 1.0f - tmp, 0.0f, 1.0f);
    
    float2 adaptedRadius = calcSplatSize(depth);
  //  float2 adaptedDiameter = adaptedRadius + adaptedRadius;

    float4 axis1, axis2;
    axis1.xyz = mul(adaptedRadius.x, normalize(cross(input[0].normal, float3(0, 0, 1))));
    axis2.xyz = mul(adaptedRadius.x, normalize(cross(input[0].normal, axis1.xyz)));
    axis1.w = 0;
    axis2.w = 0;

    float4 pos = input[0].pos;
   
    output.pos = mul(pos - axis1, m_wvp);
//    output.color = float4(1, 0, 0, 1); 
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos = mul(pos - axis2, m_wvp);
//    output.color = float4(1, 1, 0, 1);
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos = mul(pos + axis2, m_wvp);
 //   output.color = float4(0, 1, 0, 1);
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos = mul(pos + axis1, m_wvp);
 //   output.color = float4(0, 0, 1, 1);
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);
}

[maxvertexcount(4)]
void GS_RADIUS(point PosNorColRad input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    
    PosWorldNorColTex output;
    output.pos = mul(input[0].pos, m_wvp);
    output.posWorld = mul(input[0].pos, m_world);
    output.normal = mul(input[0].normal, (float3x3) m_world);
    output.color = input[0].color;

    float2 radius = mul(input[0].radius, g_splatradius) + g_splatSize;
    float2 diameter = radius + radius;

    output.pos.xy += float2(-1, 1) * radius; //left up
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos.y -= diameter.y; //right up
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos.xy += diameter; //left down
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos.y -= diameter.y; //right down
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);
}

[maxvertexcount(4)]
void GS_RADIUS_ORIENTED(point PosNorColRad input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
//    output.pos = mul(input[0].pos, m_wvp);
    output.posWorld = mul(input[0].pos, m_world);
    output.normal = mul(input[0].normal, (float3x3) m_world);
    output.color = input[0].color;

    float2 radius = mul(input[0].radius, g_splatradius)+g_splatSize;
    
  // input[0].normal = float3(0, 1, 0); 

    float4 axis1, axis2;
    axis1.xyz = mul(radius.x, normalize(cross(input[0].normal, float3(0, 0, 1))));
    axis2.xyz = mul(radius.x, normalize(cross(input[0].normal, axis1.xyz)));
    axis1.w = 0;
    axis2.w = 0;

    float4 pos = input[0].pos;
   
    output.pos = mul(pos - axis1, m_wvp);
//    output.color = float4(1, 0, 0, 1); 
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos = mul(pos - axis2, m_wvp);
//    output.color = float4(1, 1, 0, 1);
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos = mul(pos + axis2, m_wvp);
 //   output.color = float4(0, 1, 0, 1);
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos = mul(pos + axis1, m_wvp);
 //   output.color = float4(0, 0, 1, 1);
    output.tex.xy = float2(1, 1);
    OutStream.Append(output);
}


[maxvertexcount(4)]
void GS_ELLIPTICAL(point PosNorColEllipticalAxis input[1], inout TriangleStream<PosWorldNorColTex> OutStream)
{
    PosWorldNorColTex output;
//    output.pos = mul(input[0].pos, m_wvp);
    output.posWorld = mul(input[0].pos, m_world);
    output.normal = mul(input[0].normal, (float3x3) m_world);
    output.color = input[0].color;

    float2 rad;
    //splatradius...= cluster scale , splatSize = size of sample 
    rad.x = length(input[0].major) * g_splatradius.x + g_splatSize.x;
    rad.y = length(input[0].minor) * g_splatradius.x + g_splatSize.x;

    float4 major, minor; 
    major.xyz = mul(rad.x, normalize(input[0].major));
    minor.xyz = mul(rad.y, normalize(input[0].minor));
    major.w = 0;
    minor.w = 0;

    float4 pos = input[0].pos;
   
    output.pos = mul(pos - major, m_wvp);
//    output.color = float4(1, 0, 0, 1); 
    output.tex.xy = float2(-1, -1);
    OutStream.Append(output);

    output.pos = mul(pos - minor, m_wvp);
//    output.color = float4(1, 1, 0, 1);
    output.tex.xy = float2(1, -1);
    OutStream.Append(output);

    output.pos = mul(pos + minor, m_wvp);
 //   output.color = float4(0, 1, 0, 1);
    output.tex.xy = float2(-1, 1);
    OutStream.Append(output);

    output.pos = mul(pos + major, m_wvp);
 //   output.color = float4(0, 0, 1, 1);
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

float4 PS_QUADELLIPSE_NOLIGHT(PosWorldNorColTexRadXY input) : SV_TARGET
{
    return input.color;
}

float4 PS_QUADELLIPSE_PHONG(PosWorldNorColTexRadXY input) : SV_TARGET
{
    return lightning_phong(input.posWorld, input.normal) * input.color;
}

float4 PS_ELLIPSE_NOLIGHT(PosWorldNorColTex input) : SV_TARGET
{
    //point in ellipse checl 
    float2 val = input.tex * input.tex;
    if(val.x + val.y > 1.0f)
    {
        discard; 
        return float4(0, 0, 0, 0);
    }
    return input.color;
}


float4 PS_ELLIPSE_PHONG(PosWorldNorColTex input) : SV_TARGET
{
    float2 val = input.tex * input.tex;
    if (val.x + val.y > 1.0f)
    {
        discard;
        return float4(0, 0, 0, 0);
    }
    return lightning_phong(input.posWorld, input.normal) * input.color;
}
