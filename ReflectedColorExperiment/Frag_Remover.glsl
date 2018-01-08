#version 330 core

layout(location = 0) out vec3 color;
//	calibration constants
uniform sampler2DRect mapXSampler;	//	cam to pro
uniform sampler2DRect mapYSampler;
uniform vec3 gamma_p;
uniform vec3 gamma_c;
uniform vec3 c_9;
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
uniform vec2 proSize;
//	switch
uniform int rendermode;
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

float min(float a, float b)
{
	return (a < b) ? a : b;
}

vec3 min(vec3 v1, vec3 v2)
{
	return vec3(min(v1[0], v2[0]), min(v1[1], v2[1]), min(v1[2], v2[2]));
}

void main()
{
	//	step 1: 対応するカメラ画像座標にアクセス
	vec2 p_cam = vec2(gl_FragCoord.x, camSize.y - gl_FragCoord.y);
	vec2 p_pro = vec2(texture(mapXSampler, p_cam).r, texture(mapYSampler, p_cam).r);

	//	step 2: カメラ色空間での現在のプロジェクタ投影色を求める

}