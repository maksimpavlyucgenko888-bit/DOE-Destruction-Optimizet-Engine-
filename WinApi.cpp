#pragma once
#define NOMINMAX
#include "OmegaEngine.cpp"
#include <windows.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

HWND g_hwnd = NULL;
ID3D11Device* g_pd3dDevice = NULL;
ID3D11DeviceContext* g_pImmediateContext = NULL;
IDXGISwapChain* g_pSwapChain = NULL;
ID3D11RenderTargetView* g_pRenderTargetView = NULL;
ID3D11DepthStencilView* g_pDepthStencilView = NULL;
ID3D11VertexShader* g_pVertexShader = NULL;
ID3D11PixelShader* g_pPixelShader = NULL;
ID3D11InputLayout* g_pVertexLayout = NULL;
ID3D11Buffer* g_pVertexBuffer = NULL;
ID3D11Buffer* g_pConstantBuffer = NULL;

ID3D11Texture2D* g_pShadowMap = NULL;
ID3D11DepthStencilView* g_pShadowDSV = NULL;
ID3D11ShaderResourceView* g_pShadowSRV = NULL;
ID3D11SamplerState* g_pShadowSampler = NULL;
ID3D11VertexShader* g_pShadowVS = NULL;
ID3D11RasterizerState* g_pShadowRS = NULL;
ID3D11RasterizerState* g_pDefaultRS = NULL;

bool g_isCursorLocked = true;
bool g_jumpRequest = false;
bool g_isFullscreen = false;

struct Vertex3D {
    float x, y, z;
    float nx, ny, nz;
    float r, g, b, a;
};

struct CBMatrix {
    std::array<float, 16> mvp;
    std::array<float, 16> lightVP;
};

const char* g_shader3D =
"cbuffer ConstantBuffer : register(b0) { matrix MVP; matrix LightVP; };"
"Texture2D ShadowMap : register(t0);"
"SamplerComparisonState ShadowSampler : register(s0);"
"struct VS_OUT { float4 Pos : SV_POSITION; float3 Normal : NORMAL; float4 Color : COLOR0; float4 LightPos : TEXCOORD0; };"
"VS_OUT VS(float3 pos : POSITION, float3 norm : NORMAL, float4 color : COLOR) {"
"    VS_OUT o;"
"    o.Pos = mul(float4(pos, 1.0f), MVP);"
"    o.LightPos = mul(float4(pos, 1.0f), LightVP);"
"    o.Normal = norm;"
"    o.Color = color;"
"    return o;"
"}"
"float4 VS_Shadow(float3 pos : POSITION, float3 norm : NORMAL, float4 color : COLOR) : SV_POSITION {"
"    return mul(float4(pos, 1.0f), LightVP);"
"}"
"float4 PS(VS_OUT i) : SV_Target {"
"    float3 n = normalize(i.Normal);"
"    float3 sunDir = normalize(float3(45.0f, 32.0f, -35.0f));"
"    float ndotl = dot(n, sunDir);"
"    float diffuse = saturate(ndotl);"
"    float3 proj = i.LightPos.xyz / i.LightPos.w;"
"    proj.x = proj.x * 0.5f + 0.5f;"
"    proj.y = -proj.y * 0.5f + 0.5f;"
"    float shadow = 1.0f;"
"    if (ndotl > 0.02f && proj.x >= 0.0f && proj.x <= 1.0f && proj.y >= 0.0f && proj.y <= 1.0f && proj.z <= 1.0f) {"
"        float bias = 0.00018f;"
"        float sum = 0.0f;"
"        float texel = 0.75f / 4096.0f;"
"        [unroll] for (int x = -1; x <= 1; ++x) {"
"            [unroll] for (int y = -1; y <= 1; ++y) {"
"                sum += ShadowMap.SampleCmpLevelZero(ShadowSampler, proj.xy + float2(x, y) * texel, proj.z - bias);"
"            }"
"        }"
"        shadow = sum / 9.0f;"
"    } else if (ndotl <= 0.02f) {"
"        shadow = 0.0f;"
"    }"
"    float lightFactor = 0.68f + 0.32f * (diffuse * shadow);"
"    return float4(i.Color.rgb * lightFactor, i.Color.a);"
"}";

void MultiplyMat4(std::array<float, 16>& out, const std::array<float, 16>& a, const std::array<float, 16>& b) {
    std::array<float, 16> tmp;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            tmp.at(r * 4 + c) =
                a.at(r * 4 + 0) * b.at(0 * 4 + c) +
                a.at(r * 4 + 1) * b.at(1 * 4 + c) +
                a.at(r * 4 + 2) * b.at(2 * 4 + c) +
                a.at(r * 4 + 3) * b.at(3 * 4 + c);
        }
    }
    out = tmp;
}

void BuildViewProjMatrix(std::array<float, 16>& outMVP, float aspect) {
    float dx, dy, dz;
    g_engine.getLookDir(dx, dy, dz);

    float fx = dx, fy = dy, fz = dz;
    float fLen = std::sqrt(fx * fx + fy * fy + fz * fz);
    fx /= fLen; fy /= fLen; fz /= fLen;

    float upX = 0.0f, upY = 1.0f, upZ = 0.0f;
    float rx = upY * fz - upZ * fy;
    float ry = upZ * fx - upX * fz;
    float rz = upX * fy - upY * fx;
    float rLen = std::sqrt(rx * rx + ry * ry + rz * rz);
    rx /= rLen; ry /= rLen; rz /= rLen;

    float ux = fy * rz - fz * ry;
    float uy = fz * rx - fx * rz;
    float uz = fx * ry - fy * rx;

    float px = g_engine.player.x;
    float py = g_engine.player.y;
    float pz = g_engine.player.z;

    std::array<float, 16> view = {
        rx,  ux,  fx,  0.0f,
        ry,  uy,  fy,  0.0f,
        rz,  uz,  fz,  0.0f,
        -(rx * px + ry * py + rz * pz),
        -(ux * px + uy * py + uz * pz),
        -(fx * px + fy * py + fz * pz),
        1.0f
    };

    float fov = 1.25f;
    float yScale = 1.0f / std::tan(fov * 0.5f);
    float xScale = yScale / aspect;
    float zn = 0.1f, zf = 2500.0f;

    std::array<float, 16> proj = {
        xScale, 0.0f,   0.0f,                  0.0f,
        0.0f,   yScale, 0.0f,                  0.0f,
        0.0f,   0.0f,   zf / (zf - zn),        1.0f,
        0.0f,   0.0f,   -zn * zf / (zf - zn),  0.0f
    };

    std::array<float, 16> vp;
    MultiplyMat4(vp, view, proj);

    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            outMVP.at(c * 4 + r) = vp.at(r * 4 + c);
}

void BuildLightMatrix(std::array<float, 16>& outLightVP) {
    float cx = g_engine.player.x;
    float cz = g_engine.player.z;

    float sunX = cx + 45.0f, sunY = 32.0f, sunZ = cz - 35.0f;
    float fx = cx - sunX, fy = 0.0f - sunY, fz = cz - sunZ;
    float fLen = std::sqrt(fx * fx + fy * fy + fz * fz);
    fx /= fLen; fy /= fLen; fz /= fLen;

    float upX = 0.0f, upY = 1.0f, upZ = 0.0f;
    float rx = upY * fz - upZ * fy;
    float ry = upZ * fx - upX * fz;
    float rz = upX * fy - upY * fx;
    float rLen = std::sqrt(rx * rx + ry * ry + rz * rz);
    rx /= rLen; ry /= rLen; rz /= rLen;

    float ux = fy * rz - fz * ry;
    float uy = fz * rx - fx * rz;
    float uz = fx * ry - fy * rx;

    std::array<float, 16> view = {
        rx,  ux,  fx,  0.0f,
        ry,  uy,  fy,  0.0f,
        rz,  uz,  fz,  0.0f,
        -(rx * sunX + ry * sunY + rz * sunZ),
        -(ux * sunX + uy * sunY + uz * sunZ),
        -(fx * sunX + fy * sunY + fz * sunZ),
        1.0f
    };

    float size = 65.0f;
    float zn = 10.0f, zf = 110.0f;
    std::array<float, 16> ortho = {
        2.0f / size, 0.0f,        0.0f,             0.0f,
        0.0f,        2.0f / size, 0.0f,             0.0f,
        0.0f,        0.0f,        1.0f / (zf - zn), 0.0f,
        0.0f,        0.0f,       -zn / (zf - zn),   1.0f
    };

    std::array<float, 16> lvp;
    MultiplyMat4(lvp, view, ortho);

    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            outLightVP.at(c * 4 + r) = lvp.at(r * 4 + c);
}
const std::array<float, 12> IDENTITY_ROT = { 1,0,0,0, 0,1,0,0, 0,0,1,0 };

void PushRotatedBox(std::vector<Vertex3D>& v, float cx, float cy, float cz,
                    const std::array<float, 12>& rot, float hw, float hh, float hd, float r, float g, float b) {
    auto tf = [&](float lx, float ly, float lz, float& ox, float& oy, float& oz) {
        ApplyBodyMatrix(lx, ly, lz, rot, ox, oy, oz);
        ox += cx; oy += cy; oz += cz;
    };

    auto addQuad = [&](float lnx, float lny, float lnz,
                       float lx0, float ly0, float lz0, float lx1, float ly1, float lz1,
                       float lx2, float ly2, float lz2, float lx3, float ly3, float lz3) {
        float nx, ny, nz;
        ApplyBodyMatrix(lnx, lny, lnz, rot, nx, ny, nz);

        float x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3;
        tf(lx0, ly0, lz0, x0, y0, z0);
        tf(lx1, ly1, lz1, x1, y1, z1);
        tf(lx2, ly2, lz2, x2, y2, z2);
        tf(lx3, ly3, lz3, x3, y3, z3);

        v.push_back({x0, y0, z0, nx, ny, nz, r, g, b, 1.0f});
        v.push_back({x1, y1, z1, nx, ny, nz, r, g, b, 1.0f});
        v.push_back({x2, y2, z2, nx, ny, nz, r, g, b, 1.0f});
        v.push_back({x0, y0, z0, nx, ny, nz, r, g, b, 1.0f});
        v.push_back({x2, y2, z2, nx, ny, nz, r, g, b, 1.0f});
        v.push_back({x3, y3, z3, nx, ny, nz, r, g, b, 1.0f});
    };

    addQuad( 0.0f,  1.0f,  0.0f, -hw,  hh, -hd, -hw,  hh,  hd,  hw,  hh,  hd,  hw,  hh, -hd);
    addQuad( 0.0f, -1.0f,  0.0f, -hw, -hh,  hd, -hw, -hh, -hd,  hw, -hh, -hd,  hw, -hh,  hd);
    addQuad( 0.0f,  0.0f, -1.0f, -hw, -hh, -hd, -hw,  hh, -hd,  hw,  hh, -hd,  hw, -hh, -hd);
    addQuad( 0.0f,  0.0f,  1.0f,  hw, -hh,  hd,  hw,  hh,  hd, -hw,  hh,  hd, -hw, -hh,  hd);
    addQuad(-1.0f,  0.0f,  0.0f, -hw, -hh,  hd, -hw,  hh,  hd, -hw,  hh, -hd, -hw, -hh, -hd);
    addQuad( 1.0f,  0.0f,  0.0f,  hw, -hh, -hd,  hw,  hh, -hd,  hw,  hh,  hd,  hw, -hh,  hd);
}

void PushRotatedCylinder(std::vector<Vertex3D>& v, float cx, float cy, float cz,
                         const std::array<float, 12>& rot, float radius, float halfLen, float r, float g, float b) {
    const int SEGMENTS = 12;
    auto tf = [&](float lx, float ly, float lz, float& ox, float& oy, float& oz) {
        ApplyBodyMatrix(lx, ly, lz, rot, ox, oy, oz);
        ox += cx; oy += cy; oz += cz;
    };

    float fcx, fcy, fcz, bcx, bcy, bcz;
    tf(0.0f, 0.0f,  halfLen, fcx, fcy, fcz);
    tf(0.0f, 0.0f, -halfLen, bcx, bcy, bcz);

    float fnx, fny, fnz, bnx, bny, bnz;
    ApplyBodyMatrix(0.0f, 0.0f,  1.0f, rot, fnx, fny, fnz);
    ApplyBodyMatrix(0.0f, 0.0f, -1.0f, rot, bnx, bny, bnz);

    for (int i = 0; i < SEGMENTS; ++i) {
        float a0 = (2.0f * 3.1415926f * i) / SEGMENTS;
        float a1 = (2.0f * 3.1415926f * (i + 1)) / SEGMENTS;

        float c0 = std::cos(a0), s0 = std::sin(a0);
        float c1 = std::cos(a1), s1 = std::sin(a1);

        float lx0 = c0 * radius, ly0 = s0 * radius;
        float lx1 = c1 * radius, ly1 = s1 * radius;

        float fx0, fy0, fz0, fx1, fy1, fz1;
        float bx0, by0, bz0, bx1, by1, bz1;
        tf(lx0, ly0,  halfLen, fx0, fy0, fz0);
        tf(lx1, ly1,  halfLen, fx1, fy1, fz1);
        tf(lx0, ly0, -halfLen, bx0, by0, bz0);
        tf(lx1, ly1, -halfLen, bx1, by1, bz1);

        float nx0, ny0, nz0, nx1, ny1, nz1;
        ApplyBodyMatrix(c0, s0, 0.0f, rot, nx0, ny0, nz0);
        ApplyBodyMatrix(c1, s1, 0.0f, rot, nx1, ny1, nz1);

        v.push_back({bx0, by0, bz0, nx0, ny0, nz0, r, g, b, 1.0f});
        v.push_back({fx1, fy1, fz1, nx1, ny1, nz1, r, g, b, 1.0f});
        v.push_back({fx0, fy0, fz0, nx0, ny0, nz0, r, g, b, 1.0f});
        v.push_back({bx0, by0, bz0, nx0, ny0, nz0, r, g, b, 1.0f});
        v.push_back({bx1, by1, bz1, nx1, ny1, nz1, r, g, b, 1.0f});
        v.push_back({fx1, fy1, fz1, nx1, ny1, nz1, r, g, b, 1.0f});

        v.push_back({fcx, fcy, fcz, fnx, fny, fnz, r, g, b, 1.0f});
        v.push_back({fx0, fy0, fz0, fnx, fny, fnz, r, g, b, 1.0f});
        v.push_back({fx1, fy1, fz1, fnx, fny, fnz, r, g, b, 1.0f});

        v.push_back({bcx, bcy, bcz, bnx, bny, bnz, r, g, b, 1.0f});
        v.push_back({bx1, by1, bz1, bnx, bny, bnz, r, g, b, 1.0f});
        v.push_back({bx0, by0, bz0, bnx, bny, bnz, r, g, b, 1.0f});
    }
}

bool InitD3D(HWND hwnd) {
    RECT rc; GetClientRect(hwnd, &rc);
    UINT width = rc.right - rc.left; UINT height = rc.bottom - rc.top;

    DXGI_SWAP_CHAIN_DESC sd = {};
    sd.BufferCount = 2;
    sd.BufferDesc.Width = width;
    sd.BufferDesc.Height = height;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hwnd;
    sd.SampleDesc.Count = 1;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;

    D3D_FEATURE_LEVEL fl = D3D_FEATURE_LEVEL_11_0;
    if (FAILED(D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0,
        &fl, 1, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, NULL, &g_pImmediateContext))) return false;

    ID3D11Texture2D* pBack = NULL;
    g_pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (void**)&pBack);
    g_pd3dDevice->CreateRenderTargetView(pBack, NULL, &g_pRenderTargetView);
    pBack->Release();

    D3D11_TEXTURE2D_DESC depthDesc = {};
    depthDesc.Width = width; depthDesc.Height = height; depthDesc.MipLevels = 1; depthDesc.ArraySize = 1;
    depthDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT; depthDesc.SampleDesc.Count = 1;
    depthDesc.Usage = D3D11_USAGE_DEFAULT; depthDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
    ID3D11Texture2D* pDepthTex = NULL;
    g_pd3dDevice->CreateTexture2D(&depthDesc, NULL, &pDepthTex);
    g_pd3dDevice->CreateDepthStencilView(pDepthTex, NULL, &g_pDepthStencilView);
    pDepthTex->Release();

    D3D11_TEXTURE2D_DESC smDesc = {};
    smDesc.Width = 4096; smDesc.Height = 4096; smDesc.MipLevels = 1; smDesc.ArraySize = 1;
    smDesc.Format = DXGI_FORMAT_R24G8_TYPELESS; smDesc.SampleDesc.Count = 1;
    smDesc.Usage = D3D11_USAGE_DEFAULT;
    smDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL | D3D11_BIND_SHADER_RESOURCE;
    g_pd3dDevice->CreateTexture2D(&smDesc, NULL, &g_pShadowMap);

    D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
    dsvDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
    dsvDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
    g_pd3dDevice->CreateDepthStencilView(g_pShadowMap, &dsvDesc, &g_pShadowDSV);

    D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.Format = DXGI_FORMAT_R24_UNORM_X8_TYPELESS;
    srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Texture2D.MipLevels = 1;
    g_pd3dDevice->CreateShaderResourceView(g_pShadowMap, &srvDesc, &g_pShadowSRV);

    D3D11_SAMPLER_DESC sampDesc = {};
    sampDesc.Filter = D3D11_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT;
    sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_BORDER;
    sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_BORDER;
    sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_BORDER;
    *(sampDesc.BorderColor + 0) = 1.0f;
    *(sampDesc.BorderColor + 1) = 1.0f;
    *(sampDesc.BorderColor + 2) = 1.0f;
    *(sampDesc.BorderColor + 3) = 1.0f;
    sampDesc.ComparisonFunc = D3D11_COMPARISON_LESS_EQUAL;
    g_pd3dDevice->CreateSamplerState(&sampDesc, &g_pShadowSampler);

    D3D11_RASTERIZER_DESC rsDef = {};
    rsDef.FillMode = D3D11_FILL_SOLID;
    rsDef.CullMode = D3D11_CULL_BACK;
    rsDef.FrontCounterClockwise = FALSE;
    rsDef.DepthClipEnable = TRUE;
    g_pd3dDevice->CreateRasterizerState(&rsDef, &g_pDefaultRS);

    D3D11_RASTERIZER_DESC rsShad = {};
    rsShad.FillMode = D3D11_FILL_SOLID;
    rsShad.CullMode = D3D11_CULL_BACK;
    rsShad.FrontCounterClockwise = FALSE;
    rsShad.DepthClipEnable = TRUE;
    rsShad.DepthBias = 60;
    rsShad.SlopeScaledDepthBias = 0.8f;
    g_pd3dDevice->CreateRasterizerState(&rsShad, &g_pShadowRS);

    ID3DBlob* vsBlob = NULL; ID3DBlob* psBlob = NULL; ID3DBlob* shVsBlob = NULL;
    D3DCompile(g_shader3D, strlen(g_shader3D), NULL, NULL, NULL, "VS", "vs_4_0", 0, 0, &vsBlob, NULL);
    D3DCompile(g_shader3D, strlen(g_shader3D), NULL, NULL, NULL, "PS", "ps_4_0", 0, 0, &psBlob, NULL);
    D3DCompile(g_shader3D, strlen(g_shader3D), NULL, NULL, NULL, "VS_Shadow", "vs_4_0", 0, 0, &shVsBlob, NULL);

    g_pd3dDevice->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), NULL, &g_pVertexShader);
    g_pd3dDevice->CreatePixelShader(psBlob->GetBufferPointer(), psBlob->GetBufferSize(), NULL, &g_pPixelShader);
    g_pd3dDevice->CreateVertexShader(shVsBlob->GetBufferPointer(), shVsBlob->GetBufferSize(), NULL, &g_pShadowVS);

    D3D11_INPUT_ELEMENT_DESC layout[] = {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        { "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 }
    };
    g_pd3dDevice->CreateInputLayout(layout, 3, vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), &g_pVertexLayout);
    vsBlob->Release(); psBlob->Release(); shVsBlob->Release();

    D3D11_BUFFER_DESC vbd = {};
    vbd.Usage = D3D11_USAGE_DYNAMIC; vbd.ByteWidth = sizeof(Vertex3D) * 262144;
    vbd.BindFlags = D3D11_BIND_VERTEX_BUFFER; vbd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    g_pd3dDevice->CreateBuffer(&vbd, NULL, &g_pVertexBuffer);

    D3D11_BUFFER_DESC cbd = {};
    cbd.Usage = D3D11_USAGE_DEFAULT; cbd.ByteWidth = sizeof(CBMatrix);
    cbd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    g_pd3dDevice->CreateBuffer(&cbd, NULL, &g_pConstantBuffer);

    return true;
}

void RenderFrame() {
    std::vector<Vertex3D> verts;

    MaterialProps borderMat = g_materials.getMaterial(4);
    float bH = 0.25f, bT = 0.2f;
    PushRotatedBox(verts, 0.0f, bH, ARENA_HALF_SIZE + bT, IDENTITY_ROT, ARENA_HALF_SIZE + bT * 2, bH, bT, borderMat.r, borderMat.g, borderMat.b);
    PushRotatedBox(verts, 0.0f, bH, -ARENA_HALF_SIZE - bT, IDENTITY_ROT, ARENA_HALF_SIZE + bT * 2, bH, bT, borderMat.r, borderMat.g, borderMat.b);
    PushRotatedBox(verts, ARENA_HALF_SIZE + bT, bH, 0.0f, IDENTITY_ROT, bT, bH, ARENA_HALF_SIZE, borderMat.r, borderMat.g, borderMat.b);
    PushRotatedBox(verts, -ARENA_HALF_SIZE - bT, bH, 0.0f, IDENTITY_ROT, bT, bH, ARENA_HALF_SIZE, borderMat.r, borderMat.g, borderMat.b);

    for (int i = 0; i < (int)g_engine.bodies.size(); ++i) {
        const auto& b = g_engine.bodies.at(i);
        MaterialProps mat = g_materials.getMaterial(b.matId);
        float hl = (i == g_engine.heldIndex) ? 1.18f : 1.0f;

        if (b.isCylinder) {
            PushRotatedCylinder(verts, b.x, b.y, b.z, b.rot,
                                b.halfW, b.halfD, mat.r * hl, mat.g * hl, mat.b * hl);
        } else {
            PushRotatedBox(verts, b.x, b.y, b.z, b.rot,
                           b.halfW, b.halfH, b.halfD, mat.r * hl, mat.g * hl, mat.b * hl);
        }
    }

    size_t shadowVertCount = (verts.size() < 262144) ? verts.size() : 262144;

    MaterialProps groundMat = g_materials.getMaterial(0);
    PushRotatedBox(verts, 0.0f, -0.25f, 0.0f, IDENTITY_ROT,
                   ARENA_HALF_SIZE, 0.25f, ARENA_HALF_SIZE, groundMat.r, groundMat.g, groundMat.b);

    float dx, dy, dz;
    g_engine.getLookDir(dx, dy, dz);
    float aimX = g_engine.player.x + dx * 0.6f;
    float aimY = g_engine.player.y + dy * 0.6f;
    float aimZ = g_engine.player.z + dz * 0.6f;
    PushRotatedBox(verts, aimX, aimY, aimZ, IDENTITY_ROT, 0.0025f, 0.0025f, 0.0025f, 1.0f, 1.0f, 1.0f);

    size_t drawCount = (verts.size() < 262144) ? verts.size() : 262144;
    D3D11_MAPPED_SUBRESOURCE mapped;
    if (SUCCEEDED(g_pImmediateContext->Map(g_pVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
        memcpy(mapped.pData, verts.data(), drawCount * sizeof(Vertex3D));
        g_pImmediateContext->Unmap(g_pVertexBuffer, 0);
    }

    RECT rc; GetClientRect(g_hwnd, &rc);
    long h = (rc.bottom - rc.top > 1L) ? (rc.bottom - rc.top) : 1L;
    float aspect = (float)(rc.right - rc.left) / (float)h;

    CBMatrix cb;
    BuildViewProjMatrix(cb.mvp, aspect);
    BuildLightMatrix(cb.lightVP);
    g_pImmediateContext->UpdateSubresource(g_pConstantBuffer, 0, NULL, &cb, 0, 0);

    UINT stride = sizeof(Vertex3D), offset = 0;
    g_pImmediateContext->IASetVertexBuffers(0, 1, &g_pVertexBuffer, &stride, &offset);
    g_pImmediateContext->IASetInputLayout(g_pVertexLayout);
    g_pImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    ID3D11ShaderResourceView* nullSRV = NULL;
    g_pImmediateContext->PSSetShaderResources(0, 1, &nullSRV);
    g_pImmediateContext->OMSetRenderTargets(0, NULL, g_pShadowDSV);
    g_pImmediateContext->ClearDepthStencilView(g_pShadowDSV, D3D11_CLEAR_DEPTH, 1.0f, 0);

    D3D11_VIEWPORT smVP = { 0.0f, 0.0f, 4096.0f, 4096.0f, 0.0f, 1.0f };
    g_pImmediateContext->RSSetViewports(1, &smVP);
    g_pImmediateContext->RSSetState(g_pShadowRS);
    g_pImmediateContext->VSSetShader(g_pShadowVS, NULL, 0);
    g_pImmediateContext->VSSetConstantBuffers(0, 1, &g_pConstantBuffer);
    g_pImmediateContext->PSSetShader(NULL, NULL, 0);
    g_pImmediateContext->Draw((UINT)shadowVertCount, 0);

    g_pImmediateContext->OMSetRenderTargets(1, &g_pRenderTargetView, g_pDepthStencilView);
    float sky[] = { 0.53f, 0.81f, 0.92f, 1.0f };
    g_pImmediateContext->ClearRenderTargetView(g_pRenderTargetView, sky);
    g_pImmediateContext->ClearDepthStencilView(g_pDepthStencilView, D3D11_CLEAR_DEPTH, 1.0f, 0);

    D3D11_VIEWPORT vp = { 0.0f, 0.0f, (FLOAT)(rc.right - rc.left), (FLOAT)h, 0.0f, 1.0f };
    g_pImmediateContext->RSSetViewports(1, &vp);
    g_pImmediateContext->RSSetState(g_pDefaultRS);

    g_pImmediateContext->VSSetShader(g_pVertexShader, NULL, 0);
    g_pImmediateContext->VSSetConstantBuffers(0, 1, &g_pConstantBuffer);
    g_pImmediateContext->PSSetShader(g_pPixelShader, NULL, 0);
    g_pImmediateContext->PSSetShaderResources(0, 1, &g_pShadowSRV);
    g_pImmediateContext->PSSetSamplers(0, 1, &g_pShadowSampler);

    g_pImmediateContext->Draw((UINT)drawCount, 0);
    g_pSwapChain->Present(0, 0);
}

WINDOWPLACEMENT g_wpPrev = { sizeof(g_wpPrev) };

void ResizeD3D(UINT width, UINT height) {
    if (!g_pSwapChain || width == 0 || height == 0) return;

    g_pImmediateContext->OMSetRenderTargets(0, NULL, NULL);
    if (g_pRenderTargetView) { g_pRenderTargetView->Release(); g_pRenderTargetView = NULL; }
    if (g_pDepthStencilView) { g_pDepthStencilView->Release(); g_pDepthStencilView = NULL; }

    g_pSwapChain->ResizeBuffers(0, width, height, DXGI_FORMAT_UNKNOWN, 0);

    ID3D11Texture2D* pBack = NULL;
    if (SUCCEEDED(g_pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (void**)&pBack))) {
        g_pd3dDevice->CreateRenderTargetView(pBack, NULL, &g_pRenderTargetView);
        pBack->Release();
    }

    D3D11_TEXTURE2D_DESC depthDesc = {};
    depthDesc.Width = width; depthDesc.Height = height;
    depthDesc.MipLevels = 1; depthDesc.ArraySize = 1;
    depthDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
    depthDesc.SampleDesc.Count = 1;
    depthDesc.Usage = D3D11_USAGE_DEFAULT;
    depthDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;

    ID3D11Texture2D* pDepthTex = NULL;
    if (SUCCEEDED(g_pd3dDevice->CreateTexture2D(&depthDesc, NULL, &pDepthTex))) {
        g_pd3dDevice->CreateDepthStencilView(pDepthTex, NULL, &g_pDepthStencilView);
        pDepthTex->Release();
    }
}

void ToggleFullscreen(HWND hwnd) {
    DWORD dwStyle = GetWindowLong(hwnd, GWL_STYLE);
    if (!g_isFullscreen) {
        MONITORINFO mi = { sizeof(mi) };
        if (GetWindowPlacement(hwnd, &g_wpPrev) &&
            GetMonitorInfo(MonitorFromWindow(hwnd, MONITOR_DEFAULTTOPRIMARY), &mi)) {
            SetWindowLong(hwnd, GWL_STYLE, dwStyle & ~WS_OVERLAPPEDWINDOW);
            SetWindowPos(hwnd, HWND_TOP,
                         mi.rcMonitor.left, mi.rcMonitor.top,
                         mi.rcMonitor.right - mi.rcMonitor.left,
                         mi.rcMonitor.bottom - mi.rcMonitor.top,
                         SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
            g_isFullscreen = true;
        }
    } else {
        SetWindowLong(hwnd, GWL_STYLE, dwStyle | WS_OVERLAPPEDWINDOW);
        SetWindowPlacement(hwnd, &g_wpPrev);
        SetWindowPos(hwnd, NULL, 0, 0, 0, 0,
                     SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
        g_isFullscreen = false;
    }
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
        case WM_SIZE:
            if (g_pd3dDevice != NULL && wParam != SIZE_MINIMIZED) {
                ResizeD3D(LOWORD(lParam), HIWORD(lParam));
            }
            return 0;
        case WM_SETFOCUS:
            if (g_isCursorLocked) ShowCursor(FALSE);
            return 0;
        case WM_KILLFOCUS:
            ShowCursor(TRUE);
            return 0;
        case WM_KEYDOWN:
            if (wParam == VK_F11) {
                ToggleFullscreen(hwnd);
            }
            if (wParam == VK_ESCAPE) {
                g_isCursorLocked = !g_isCursorLocked;
                ShowCursor(!g_isCursorLocked);
            }
            if (wParam == VK_SPACE) g_jumpRequest = true;
            if (wParam == 'E' || wParam == 'e') g_engine.toggleGrab();
            if (wParam == 'R' || wParam == 'r') g_engine.resetWorld();
            if (wParam == '1') g_engine.spawnInFront(1);
            if (wParam == '2') g_engine.spawnInFront(2);
            if (wParam == '3') g_engine.spawnInFront(3);
            if (wParam == '4') g_engine.spawnInFront(4);
            if (wParam == '5') g_engine.spawnInFront(5);
            return 0;
        case WM_LBUTTONDOWN:
            if (g_isCursorLocked) g_engine.toggleGrab();
            return 0;
        case WM_DESTROY:
            ShowCursor(TRUE);
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    g_materials.loadBaseMaterials();
    g_engine.resetWorld();

    WNDCLASS wc = {};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInstance;
    wc.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(1));
    wc.lpszClassName = "DOE_EngineClass";
    RegisterClass(&wc);

    g_hwnd = CreateWindowEx(0, "DOE_EngineClass", "Destruction Optimized Engine Alpha 0.5",
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 1024, 720, NULL, NULL, hInstance, NULL);
    if (!g_hwnd || !InitD3D(g_hwnd)) return 0;

    ShowWindow(g_hwnd, nCmdShow);
    if (g_isCursorLocked) ShowCursor(FALSE);

    LARGE_INTEGER freq, lastTime;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&lastTime);

    MSG msg = {};
    while (msg.message != WM_QUIT) {
        if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        } else {
            LARGE_INTEGER now;
            QueryPerformanceCounter(&now);
            float dt = (float)(now.QuadPart - lastTime.QuadPart) / (float)freq.QuadPart;
            lastTime = now;
            if (dt > 0.05f) dt = 0.05f;

            if (g_isCursorLocked && GetActiveWindow() == g_hwnd) {
                RECT rect; GetWindowRect(g_hwnd, &rect);
                int cx = rect.left + (rect.right - rect.left) / 2;
                int cy = rect.top + (rect.bottom - rect.top) / 2;
                POINT mp; GetCursorPos(&mp);
                int dx = mp.x - cx;
                int dy = mp.y - cy;
                if (dx != 0 || dy != 0) {
                    g_engine.player.yaw += (float)dx * 0.0025f;
                    g_engine.player.pitch += (float)dy * 0.0025f;
                    if (g_engine.player.pitch < -1.45f) g_engine.player.pitch = -1.45f;
                    if (g_engine.player.pitch > 1.45f)  g_engine.player.pitch = 1.45f;
                    SetCursorPos(cx, cy);
                }
            }

            float moveF = 0.0f, moveR = 0.0f;
            if (GetActiveWindow() == g_hwnd) {
                if (GetAsyncKeyState('W') & 0x8000) moveF += 1.0f;
                if (GetAsyncKeyState('S') & 0x8000) moveF -= 1.0f;
                if (GetAsyncKeyState('D') & 0x8000) moveR += 1.0f;
                if (GetAsyncKeyState('A') & 0x8000) moveR -= 1.0f;
            }

            g_engine.updateTick(dt, moveF, moveR, g_jumpRequest);
            g_jumpRequest = false;

            RenderFrame();
        }
    }
    return 0;
}
