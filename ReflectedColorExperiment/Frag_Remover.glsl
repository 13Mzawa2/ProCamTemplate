#version 330 core

layout(location = 0) out vec3 color;
//	calibration constants
uniform sampler2DRect mapXSampler;	//	cam to pro
uniform sampler2DRect mapYSampler;
uniform vec3 gamma_p;
uniform vec3 gamma_c;
uniform vec3 c_0;
uniform vec3 c_th;
uniform mat3 cmatPC;
//	color estimation model params
uniform mat3 matJD;
uniform mat3 matJDinv;
uniform mat3 matPCA0;
uniform mat3 matPCA1;
uniform mat3 matPCA2;
//	resolution
uniform vec2 camSize;
uniform vec2 projSize;
//	image
uniform sampler2DRect camImgSampler;
uniform sampler2DRect projImgSampler;
//	switch
uniform int estmodel;

//	行列の転置
mat3 transpose(mat3 m)
{
	mat3 mt;
	mt[0][0] = m[0][0]; mt[1][0] = m[0][1]; mt[2][0] = m[0][2];
	mt[0][1] = m[1][0]; mt[1][1] = m[1][1]; mt[2][1] = m[1][2];
	mt[0][2] = m[2][0]; mt[1][2] = m[2][1]; mt[2][2] = m[2][2];

	return mt;
}

//	3x3逆行列公式
//	GLSLのmat3の3x3行列は次のように並んでいる
//	m[0][0] m[1][0] m[2][0]
//	m[0][1] m[1][1] m[2][1]
//	m[0][2] m[1][2] m[2][2]
mat3 invmat(mat3 m)
{
	mat3 mi;
	//	行列式
	float det = m[0][0] * m[1][1] * m[2][2] + m[1][0] * m[2][1] * m[0][2] + m[2][0] * m[0][1] * m[1][2]
		- m[0][0] * m[1][2] * m[2][1] - m[1][0] * m[0][1] * m[2][2] - m[2][0] * m[1][1] * m[0][2];
	//	第1列ベクトル
	mi[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det;
	mi[0][1] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det;
	mi[0][2] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det;
	//	第2列ベクトル
	mi[1][0] = (m[2][1] * m[0][2] - m[0][1] * m[2][2]) / det;
	mi[1][1] = (m[0][0] * m[2][2] - m[2][0] * m[0][2]) / det;
	mi[1][2] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) / det;
	//	第3列ベクトル
	mi[2][0] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;
	mi[2][1] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) / det;
	mi[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;

	mat3 mt = transpose(mi);
	return mt;
}

float max(float a, float b)
{
	return (a < b) ? b : a;
}

float min(float a, float b)
{
	return (a < b) ? a : b;
}

vec3 min(vec3 v1, vec3 v2)
{
	return vec3(min(v1[0], v2[0]), min(v1[1], v2[1]), min(v1[2], v2[2]));
}

//	線形化
vec3 linearizePro(vec3 P)
{
	return vec3(
		pow(P.b, gamma_p.x),
		pow(P.g, gamma_p.y),
		pow(P.r, gamma_p.z)
	);
}

vec3 linearizeCam(vec3 C)
{
	return vec3(
		pow(C.b, gamma_c.x) + c_th.x,
		pow(C.g, gamma_c.y) + c_th.y,
		pow(C.r, gamma_c.z) + c_th.z
	);
}

vec3 gammaCam(vec3 C)
{
	return vec3(
		pow(max(C.b - c_th.x, 0.001), 1.0 / gamma_c.x),
		pow(max(C.g - c_th.y, 0.001), 1.0 / gamma_c.y),
		pow(max(C.r - c_th.z, 0.001), 1.0 / gamma_c.z)
	);
}

//	反射率推定モデル
mat3 estimateR_Diag(vec3 C, vec3 Cp)
{
	mat3 R;
	for (int i = 0; i < 3; i++) {
		R[i][i] = C[i] / max(Cp[i], 0.001);
	}
	return R;
}

mat3 estimateR_PCA(vec3 C, vec3 Cp)
{
	mat3 R,U;
	mat3 matPCA[] = mat3[](matPCA0, matPCA1, matPCA2);	//	matPCA[k][j][i]
	for (int i = 0; i < 3; i++) {
		for (int k = 0; k < 3; k++) {
			U[k][i] = matPCA[k][0][i] * Cp[0] + matPCA[k][1][i] * Cp[1] + matPCA[k][2][i] * Cp[2];
		}
	}
	vec3 alpha = invmat(U)*C;
	R = alpha[0] * matPCA[0] + alpha[1] * matPCA[1] + alpha[2] * matPCA[2];
	return R;
}

mat3 estimateR_JD(vec3 C, vec3 Cp)
{
	mat3 R, Rd;
	vec3 C_d = matJDinv*C;
	vec3 Cp_d = matJDinv*Cp;
	Rd = estimateR_Diag(C_d, Cp_d);
	R = matJD * Rd * matJDinv;
	return R;
}

void main()
{
	//	step 1: 対応するカメラ画像座標にアクセス
	vec2 p_cam = vec2(gl_FragCoord.x, camSize.y - gl_FragCoord.y);
	vec2 p_pro = vec2(texture(mapXSampler, p_cam).r, texture(mapYSampler, p_cam).r);

	//	step 2: カメラ色空間での現在のプロジェクタ投影色を求める
	vec3 proColor = texture(projImgSampler, p_pro).bgr;
	vec3 camColor = texture(camImgSampler, p_cam).bgr;
	vec3 Cp = cmatPC * linearizePro(proColor) + c_0;
	vec3 C = linearizeCam(camColor);

	//	step 3: 反射率の推定
	mat3 R;
	switch (estmodel) {
	case 1:	//	PCA
		R = estimateR_PCA(C, Cp);
		break;
	case 2: //	JD
		R = estimateR_JD(C, Cp);
		break;
	default:
		R = estimateR_Diag(C, Cp);
		break;
	}

	//	step 4: 白色光投影時の色を推定
	vec3 Cp_w = cmatPC * vec3(1.0, 1.0, 1.0) + c_0;
	vec3 C_w = R*Cp_w;
	vec3 camColor_est = gammaCam(C_w);

	color = camColor_est;

	color = vec3(1.0,1.0,0.0);
}