/*
 * math_calc.c
 *
 *  Created on: May 14, 2020
 *      Author: iwai
 */
#include <stdio.h>
#include <math.h>

static const float SAMPLE_NUM_F = 1000.f;
static const int SAMPLE_NUM_I = 1000;

static const float micro_10 = 0.000001f;
static const float PI2 = 6.283185307f;
static const float SIN_PLACE = 159.154943096f;// = 1000 / 6.283185307

uint64_t get_system_count_in_us();


static float sin_table[1001] = {
		1.000000, 1.006283, 1.012566, 1.018848, 1.025130, 1.031411, 1.037690, 1.043968, 1.050244, 1.056519,
		1.062791, 1.069060, 1.075327, 1.081591, 1.087851, 1.094108, 1.100362, 1.106611, 1.112856, 1.119097,
		1.125333, 1.131564, 1.137790, 1.144011, 1.150226, 1.156434, 1.162637, 1.168833, 1.175023, 1.181206,
		1.187381, 1.193549, 1.199710, 1.205863, 1.212007, 1.218143, 1.224271, 1.230389, 1.236499, 1.242599,
		1.248690, 1.254771, 1.260841, 1.266902, 1.272952, 1.278991, 1.285019, 1.291036, 1.297042, 1.303035,
		1.309017, 1.314986, 1.320944, 1.326888, 1.332820, 1.338738, 1.344643, 1.350534, 1.356412, 1.362275,
		1.368125, 1.373959, 1.379779, 1.385584, 1.391374, 1.397148, 1.402907, 1.408649, 1.414376, 1.420086,
		1.425779, 1.431456, 1.437116, 1.442758, 1.448383, 1.453991, 1.459580, 1.465151, 1.470704, 1.476238,
		1.481754, 1.487250, 1.492728, 1.498185, 1.503623, 1.509042, 1.514440, 1.519817, 1.525175, 1.530511,
		1.535827, 1.541121, 1.546394, 1.551646, 1.556876, 1.562083, 1.567269, 1.572432, 1.577573, 1.582690,
		1.587785, 1.592857, 1.597905, 1.602929, 1.607930, 1.612907, 1.617859, 1.622787, 1.627691, 1.632570,
		1.637424, 1.642252, 1.647056, 1.651833, 1.656585, 1.661312, 1.666011, 1.670685, 1.675332, 1.679953,
		1.684547, 1.689113, 1.693653, 1.698165, 1.702649, 1.707106, 1.711535, 1.715936, 1.720308, 1.724653,
		1.728968, 1.733255, 1.737512, 1.741741, 1.745941, 1.750110, 1.754251, 1.758361, 1.762442, 1.766492,
		1.770513, 1.774502, 1.778462, 1.782390, 1.786288, 1.790154, 1.793990, 1.797794, 1.801566, 1.805307,
		1.809016, 1.812693, 1.816339, 1.819951, 1.823532, 1.827080, 1.830595, 1.834078, 1.837527, 1.840944,
		1.844327, 1.847677, 1.850994, 1.854277, 1.857526, 1.860741, 1.863923, 1.867070, 1.870183, 1.873262,
		1.876306, 1.879315, 1.882290, 1.885231, 1.888136, 1.891006, 1.893841, 1.896640, 1.899404, 1.902133,
		1.904826, 1.907484, 1.910105, 1.912691, 1.915240, 1.917754, 1.920231, 1.922672, 1.925076, 1.927444,
		1.929776, 1.932070, 1.934328, 1.936549, 1.938733, 1.940880, 1.942990, 1.945062, 1.947098, 1.949095,
		1.951056, 1.952979, 1.954864, 1.956711, 1.958521, 1.960293, 1.962027, 1.963723, 1.965381, 1.967001,
		1.968583, 1.970126, 1.971631, 1.973098, 1.974526, 1.975916, 1.977268, 1.978580, 1.979854, 1.981090,
		1.982287, 1.983445, 1.984564, 1.985644, 1.986686, 1.987688, 1.988651, 1.989576, 1.990461, 1.991307,
		1.992114, 1.992882, 1.993611, 1.994300, 1.994951, 1.995562, 1.996133, 1.996666, 1.997159, 1.997612,
		1.998026, 1.998401, 1.998737, 1.999033, 1.999289, 1.999506, 1.999684, 1.999822, 1.999921, 1.999980,
		2.000000, 1.999980, 1.999921, 1.999822, 1.999684, 1.999507, 1.999290, 1.999033, 1.998737, 1.998402,
		1.998027, 1.997613, 1.997159, 1.996666, 1.996134, 1.995562, 1.994951, 1.994301, 1.993612, 1.992883,
		1.992115, 1.991308, 1.990462, 1.989577, 1.988652, 1.987689, 1.986687, 1.985645, 1.984565, 1.983446,
		1.982288, 1.981091, 1.979856, 1.978582, 1.977269, 1.975918, 1.974528, 1.973099, 1.971633, 1.970128,
		1.968584, 1.967003, 1.965383, 1.963725, 1.962029, 1.960295, 1.958523, 1.956713, 1.954866, 1.952981,
		1.951058, 1.949098, 1.947100, 1.945065, 1.942992, 1.940882, 1.938735, 1.936551, 1.934331, 1.932073,
		1.929778, 1.927447, 1.925079, 1.922675, 1.920234, 1.917757, 1.915243, 1.912694, 1.910108, 1.907486,
		1.904829, 1.902136, 1.899407, 1.896643, 1.893843, 1.891008, 1.888138, 1.885233, 1.882293, 1.879318,
		1.876308, 1.873264, 1.870185, 1.867072, 1.863925, 1.860744, 1.857528, 1.854279, 1.850996, 1.847679,
		1.844329, 1.840946, 1.837529, 1.834080, 1.830597, 1.827082, 1.823534, 1.819953, 1.816340, 1.812695,
		1.809018, 1.805309, 1.801568, 1.797795, 1.793991, 1.790156, 1.786289, 1.782391, 1.778463, 1.774504,
		1.770514, 1.766493, 1.762443, 1.758362, 1.754252, 1.750111, 1.745941, 1.741742, 1.737513, 1.733255,
		1.728969, 1.724653, 1.720309, 1.715936, 1.711535, 1.707106, 1.702649, 1.698165, 1.693653, 1.689113,
		1.684546, 1.679952, 1.675332, 1.670684, 1.666011, 1.661311, 1.656584, 1.651832, 1.647054, 1.642251,
		1.637422, 1.632568, 1.627689, 1.622786, 1.617858, 1.612905, 1.607928, 1.602927, 1.597903, 1.592854,
		1.587783, 1.582688, 1.577570, 1.572429, 1.567266, 1.562080, 1.556872, 1.551643, 1.546391, 1.541118,
		1.535823, 1.530508, 1.525171, 1.519814, 1.514436, 1.509037, 1.503619, 1.498181, 1.492723, 1.487246,
		1.481749, 1.476234, 1.470699, 1.465146, 1.459575, 1.453985, 1.448378, 1.442753, 1.437110, 1.431451,
		1.425774, 1.420080, 1.414370, 1.408643, 1.402900, 1.397142, 1.391368, 1.385578, 1.379773, 1.373953,
		1.368118, 1.362269, 1.356405, 1.350527, 1.344636, 1.338731, 1.332812, 1.326881, 1.320936, 1.314979,
		1.309009, 1.303028, 1.297034, 1.291028, 1.285011, 1.278983, 1.272944, 1.266894, 1.260833, 1.254762,
		1.248681, 1.242590, 1.236490, 1.230380, 1.224262, 1.218134, 1.211998, 1.205853, 1.199700, 1.193540,
		1.187371, 1.181196, 1.175013, 1.168823, 1.162627, 1.156424, 1.150215, 1.144000, 1.137780, 1.131554,
		1.125322, 1.119086, 1.112845, 1.106600, 1.100350, 1.094097, 1.087840, 1.081579, 1.075315, 1.069048,
		1.062779, 1.056507, 1.050232, 1.043956, 1.037678, 1.031398, 1.025118, 1.018836, 1.012553, 1.006270,
		0.999987, 0.993704, 0.987421, 0.981138, 0.974857, 0.968576, 0.962296, 0.956018, 0.949742, 0.943468,
		0.937196, 0.930926, 0.924659, 0.918395, 0.912135, 0.905877, 0.899624, 0.893374, 0.887129, 0.880888,
		0.874652, 0.868421, 0.862195, 0.855974, 0.849759, 0.843550, 0.837348, 0.831151, 0.824961, 0.818779,
		0.812603, 0.806435, 0.800274, 0.794122, 0.787977, 0.781841, 0.775713, 0.769594, 0.763485, 0.757384,
		0.751294, 0.745213, 0.739142, 0.733081, 0.727031, 0.720992, 0.714964, 0.708947, 0.702942, 0.696948,
		0.690966, 0.684996, 0.679039, 0.673095, 0.667163, 0.661245, 0.655340, 0.649448, 0.643571, 0.637707,
		0.631858, 0.626023, 0.620203, 0.614398, 0.608609, 0.602834, 0.597076, 0.591333, 0.585607, 0.579896,
		0.574203, 0.568526, 0.562866, 0.557224, 0.551599, 0.545991, 0.540402, 0.534831, 0.529278, 0.523744,
		0.518228, 0.512732, 0.507254, 0.501797, 0.496358, 0.490940, 0.485542, 0.480164, 0.474807, 0.469470,
		0.464155, 0.458860, 0.453587, 0.448336, 0.443106, 0.437898, 0.432713, 0.427549, 0.422409, 0.417291,
		0.412196, 0.407125, 0.402077, 0.397052, 0.392051, 0.387075, 0.382122, 0.377194, 0.372290, 0.367412,
		0.362558, 0.357729, 0.352926, 0.348148, 0.343396, 0.338670, 0.333970, 0.329296, 0.324649, 0.320029,
		0.315435, 0.310868, 0.306329, 0.301817, 0.297332, 0.292875, 0.288447, 0.284046, 0.279673, 0.275329,
		0.271014, 0.266727, 0.262469, 0.258241, 0.254041, 0.249872, 0.245731, 0.241621, 0.237541, 0.233490,
		0.229470, 0.225480, 0.221521, 0.217593, 0.213695, 0.209828, 0.205993, 0.202189, 0.198417, 0.194676,
		0.190967, 0.187290, 0.183645, 0.180032, 0.176452, 0.172904, 0.169389, 0.165906, 0.162457, 0.159040,
		0.155657, 0.152307, 0.148991, 0.145708, 0.142459, 0.139243, 0.136062, 0.132915, 0.129802, 0.126723,
		0.123679, 0.120670, 0.117695, 0.114755, 0.111850, 0.108980, 0.106145, 0.103346, 0.100582, 0.097853,
		0.095160, 0.092503, 0.089881, 0.087296, 0.084746, 0.082233, 0.079756, 0.077315, 0.074911, 0.072543,
		0.070212, 0.067918, 0.065660, 0.063439, 0.061255, 0.059108, 0.056999, 0.054926, 0.052891, 0.050894,
		0.048933, 0.047011, 0.045126, 0.043278, 0.041469, 0.039697, 0.037963, 0.036268, 0.034610, 0.032990,
		0.031409, 0.029865, 0.028360, 0.026894, 0.025466, 0.024076, 0.022725, 0.021412, 0.020138, 0.018903,
		0.017706, 0.016549, 0.015430, 0.014350, 0.013308, 0.012306, 0.011343, 0.010419, 0.009534, 0.008688,
		0.007881, 0.007113, 0.006385, 0.005695, 0.005045, 0.004435, 0.003863, 0.003331, 0.002838, 0.002385,
		0.001971, 0.001596, 0.001261, 0.000965, 0.000709, 0.000492, 0.000315, 0.000177, 0.000078, 0.000020,
		0.000000, 0.000020, 0.000079, 0.000178, 0.000317, 0.000495, 0.000712, 0.000969, 0.001265, 0.001601,
		0.001976, 0.002390, 0.002844, 0.003337, 0.003870, 0.004442, 0.005053, 0.005703, 0.006393, 0.007122,
		0.007890, 0.008698, 0.009544, 0.010430, 0.011354, 0.012318, 0.013321, 0.014362, 0.015443, 0.016562,
		0.017720, 0.018917, 0.020153, 0.021427, 0.022740, 0.024092, 0.025482, 0.026911, 0.028378, 0.029883,
		0.031427, 0.033009, 0.034629, 0.036287, 0.037984, 0.039718, 0.041490, 0.043300, 0.045148, 0.047033,
		0.048956, 0.050917, 0.052915, 0.054951, 0.057024, 0.059134, 0.061281, 0.063465, 0.065686, 0.067944,
		0.070239, 0.072571, 0.074939, 0.077344, 0.079785, 0.082263, 0.084776, 0.087326, 0.089912, 0.092534,
		0.095192, 0.097885, 0.100614, 0.103379, 0.106179, 0.109014, 0.111884, 0.114790, 0.117730, 0.120705,
		0.123715, 0.126760, 0.129839, 0.132952, 0.136099, 0.139281, 0.142497, 0.145746, 0.149030, 0.152346,
		0.155697, 0.159080, 0.162497, 0.165947, 0.169430, 0.172946, 0.176494, 0.180075, 0.183688, 0.187333,
		0.191011, 0.194720, 0.198461, 0.202234, 0.206038, 0.209874, 0.213741, 0.217639, 0.221568, 0.225527,
		0.229517, 0.233538, 0.237589, 0.241670, 0.245780, 0.249921, 0.254091, 0.258291, 0.262520, 0.266778,
		0.271065, 0.275381, 0.279725, 0.284098, 0.288499, 0.292928, 0.297385, 0.301870, 0.306383, 0.310922,
		0.315489, 0.320083, 0.324704, 0.329352, 0.334026, 0.338726, 0.343452, 0.348205, 0.352983, 0.357786,
		0.362615, 0.367469, 0.372348, 0.377252, 0.382181, 0.387134, 0.392111, 0.397112, 0.402136, 0.407185,
		0.412257, 0.417352, 0.422470, 0.427611, 0.432774, 0.437960, 0.443168, 0.448398, 0.453650, 0.458923,
		0.464218, 0.469534, 0.474871, 0.480228, 0.485606, 0.491005, 0.496423, 0.501861, 0.507319, 0.512797,
		0.518293, 0.523809, 0.529344, 0.534897, 0.540468, 0.546058, 0.551665, 0.557291, 0.562933, 0.568593,
		0.574270, 0.579964, 0.585675, 0.591401, 0.597144, 0.602903, 0.608677, 0.614467, 0.620272, 0.626092,
		0.631927, 0.637777, 0.643640, 0.649518, 0.655410, 0.661315, 0.667234, 0.673165, 0.679110, 0.685067,
		0.691037, 0.697019, 0.703013, 0.709018, 0.715036, 0.721064, 0.727103, 0.733153, 0.739214, 0.745285,
		0.751366, 0.757457, 0.763557, 0.769667, 0.775786, 0.781914, 0.788050, 0.794195, 0.800347, 0.806508,
		0.812676, 0.818852, 0.825035, 0.831225, 0.837421, 0.843624, 0.849833, 0.856048, 0.862269, 0.868495,
		0.874726, 0.880962, 0.887203, 0.893449, 0.899698, 0.905952, 0.912209, 0.918470, 0.924734, 0.931000,
		0.937270, 0.943542, 0.949817, 0.956093, 0.962371, 0.968650, 0.974931, 0.981213, 0.987495, 0.993778, 1.000000
};


//線形補完処理
static float lerp(float x0, float y0, float x1, float y1, float x) {
    // return -y0 + y0 + (y1 - y0) * (x - x0) / (x1 - x0) ;
    return (y1 - y0) * (x - x0) / (x1 - x0) ;
}

float calc_sin(uint32_t helz)
{
	uint64_t time = get_system_count_in_us();

    float TT = time * helz * micro_10;
    float TT_shosu = TT - (int)TT;

    float addr_f = SAMPLE_NUM_F * TT_shosu;
    int addr = (int)addr_f;

    float shosuAddr = addr_f - (int)addr;
    //引数 x0,x1は結局1の絶対値が必要なだけだから0,1とする
    float hokan = lerp(0, sin_table[addr], 1, sin_table[addr + 1], shosuAddr);

    return sin_table[addr] + hokan;
}


float calc_sin_float(float helz, float mod, uint64_t now_time_in_us)
{
    float TT = now_time_in_us * helz * micro_10;
    float R = (PI2 * (TT - (int)TT)) + mod + PI2;
//    float addr_f = R * SIN_PLACE;
//    int addr = ((int)(R * SIN_PLACE)) % SAMPLE_NUM_I;
//    addr %= SAMPLE_NUM_I;

//    float shosuAddr = addr_f - (int)addr;
//    //引数 x0,x1は結局1の絶対値が必要なだけだから0,1とする
//    float hokan = lerp(0, sin_table[addr], 1, sin_table[addr + 1], shosuAddr);
//
//    return sin_table[addr] + hokan;
    return sin_table[((int)(R * SIN_PLACE)) % SAMPLE_NUM_I];
}

