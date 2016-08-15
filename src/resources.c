#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include <math.h>
// for uint16_t, uint8_t, uint32_t
#include <stdint.h>
/*
http://www.daycounter.com/Calculators/Sine-Generator-Calculator.phtml
*/

// table for Sine with 12-bit resolution 2^12=4096, minimal 0, maximal - 4095
const uint16_t Sine1024_12bit [] =
{
		2048,2060,2073,2085,2098,2110,2123,2135,
		2148,2161,2173,2186,2198,2211,2223,2236,
		2248,2261,2273,2286,2298,2311,2323,2335,
		2348,2360,2373,2385,2398,2410,2422,2435,
		2447,2459,2472,2484,2496,2508,2521,2533,
		2545,2557,2569,2581,2594,2606,2618,2630,
		2642,2654,2666,2678,2690,2702,2714,2725,
		2737,2749,2761,2773,2784,2796,2808,2819,
		2831,2843,2854,2866,2877,2889,2900,2912,
		2923,2934,2946,2957,2968,2979,2990,3002,
		3013,3024,3035,3046,3057,3068,3078,3089,
		3100,3111,3122,3132,3143,3154,3164,3175,
		3185,3195,3206,3216,3226,3237,3247,3257,
		3267,3277,3287,3297,3307,3317,3327,3337,
		3346,3356,3366,3375,3385,3394,3404,3413,
		3423,3432,3441,3450,3459,3468,3477,3486,
		3495,3504,3513,3522,3530,3539,3548,3556,
		3565,3573,3581,3590,3598,3606,3614,3622,
		3630,3638,3646,3654,3662,3669,3677,3685,
		3692,3700,3707,3714,3722,3729,3736,3743,
		3750,3757,3764,3771,3777,3784,3791,3797,
		3804,3810,3816,3823,3829,3835,3841,3847,
		3853,3859,3865,3871,3876,3882,3888,3893,
		3898,3904,3909,3914,3919,3924,3929,3934,
		3939,3944,3949,3953,3958,3962,3967,3971,
		3975,3980,3984,3988,3992,3996,3999,4003,
		4007,4010,4014,4017,4021,4024,4027,4031,
		4034,4037,4040,4042,4045,4048,4051,4053,
		4056,4058,4060,4063,4065,4067,4069,4071,
		4073,4075,4076,4078,4080,4081,4083,4084,
		4085,4086,4087,4088,4089,4090,4091,4092,
		4093,4093,4094,4094,4094,4095,4095,4095,
		4095,4095,4095,4095,4094,4094,4094,4093,
		4093,4092,4091,4090,4089,4088,4087,4086,
		4085,4084,4083,4081,4080,4078,4076,4075,
		4073,4071,4069,4067,4065,4063,4060,4058,
		4056,4053,4051,4048,4045,4042,4040,4037,
		4034,4031,4027,4024,4021,4017,4014,4010,
		4007,4003,3999,3996,3992,3988,3984,3980,
		3975,3971,3967,3962,3958,3953,3949,3944,
		3939,3934,3929,3924,3919,3914,3909,3904,
		3898,3893,3888,3882,3876,3871,3865,3859,
		3853,3847,3841,3835,3829,3823,3816,3810,
		3804,3797,3791,3784,3777,3771,3764,3757,
		3750,3743,3736,3729,3722,3714,3707,3700,
		3692,3685,3677,3669,3662,3654,3646,3638,
		3630,3622,3614,3606,3598,3590,3581,3573,
		3565,3556,3548,3539,3530,3522,3513,3504,
		3495,3486,3477,3468,3459,3450,3441,3432,
		3423,3413,3404,3394,3385,3375,3366,3356,
		3346,3337,3327,3317,3307,3297,3287,3277,
		3267,3257,3247,3237,3226,3216,3206,3195,
		3185,3175,3164,3154,3143,3132,3122,3111,
		3100,3089,3078,3068,3057,3046,3035,3024,
		3013,3002,2990,2979,2968,2957,2946,2934,
		2923,2912,2900,2889,2877,2866,2854,2843,
		2831,2819,2808,2796,2784,2773,2761,2749,
		2737,2725,2714,2702,2690,2678,2666,2654,
		2642,2630,2618,2606,2594,2581,2569,2557,
		2545,2533,2521,2508,2496,2484,2472,2459,
		2447,2435,2422,2410,2398,2385,2373,2360,
		2348,2335,2323,2311,2298,2286,2273,2261,
		2248,2236,2223,2211,2198,2186,2173,2161,
		2148,2135,2123,2110,2098,2085,2073,2060,
		2048,2035,2022,2010,1997,1985,1972,1960,
		1947,1934,1922,1909,1897,1884,1872,1859,
		1847,1834,1822,1809,1797,1784,1772,1760,
		1747,1735,1722,1710,1697,1685,1673,1660,
		1648,1636,1623,1611,1599,1587,1574,1562,
		1550,1538,1526,1514,1501,1489,1477,1465,
		1453,1441,1429,1417,1405,1393,1381,1370,
		1358,1346,1334,1322,1311,1299,1287,1276,
		1264,1252,1241,1229,1218,1206,1195,1183,
		1172,1161,1149,1138,1127,1116,1105,1093,
		1082,1071,1060,1049,1038,1027,1017,1006,
		995,984,973,963,952,941,931,920,
		910,900,889,879,869,858,848,838,
		828,818,808,798,788,778,768,758,
		749,739,729,720,710,701,691,682,
		672,663,654,645,636,627,618,609,
		600,591,582,573,565,556,547,539,
		530,522,514,505,497,489,481,473,
		465,457,449,441,433,426,418,410,
		403,395,388,381,373,366,359,352,
		345,338,331,324,318,311,304,298,
		291,285,279,272,266,260,254,248,
		242,236,230,224,219,213,207,202,
		197,191,186,181,176,171,166,161,
		156,151,146,142,137,133,128,124,
		120,115,111,107,103,99,96,92,
		88,85,81,78,74,71,68,64,
		61,58,55,53,50,47,44,42,
		39,37,35,32,30,28,26,24,
		22,20,19,17,15,14,12,11,
		10,9,8,7,6,5,4,3,
		2,2,1,1,1,0,0,0,
		0,0,0,0,1,1,1,2,
		2,3,4,5,6,7,8,9,
		10,11,12,14,15,17,19,20,
		22,24,26,28,30,32,35,37,
		39,42,44,47,50,53,55,58,
		61,64,68,71,74,78,81,85,
		88,92,96,99,103,107,111,115,
		120,124,128,133,137,142,146,151,
		156,161,166,171,176,181,186,191,
		197,202,207,213,219,224,230,236,
		242,248,254,260,266,272,279,285,
		291,298,304,311,318,324,331,338,
		345,352,359,366,373,381,388,395,
		403,410,418,426,433,441,449,457,
		465,473,481,489,497,505,514,522,
		530,539,547,556,565,573,582,591,
		600,609,618,627,636,645,654,663,
		672,682,691,701,710,720,729,739,
		749,758,768,778,788,798,808,818,
		828,838,848,858,869,879,889,900,
		910,920,931,941,952,963,973,984,
		995,1006,1017,1027,1038,1049,1060,1071,
		1082,1093,1105,1116,1127,1138,1149,1161,
		1172,1183,1195,1206,1218,1229,1241,1252,
		1264,1276,1287,1299,1311,1322,1334,1346,
		1358,1370,1381,1393,1405,1417,1429,1441,
		1453,1465,1477,1489,1501,1514,1526,1538,
		1550,1562,1574,1587,1599,1611,1623,1636,
		1648,1660,1673,1685,1697,1710,1722,1735,
		1747,1760,1772,1784,1797,1809,1822,1834,
		1847,1859,1872,1884,1897,1909,1922,1934,
		1947,1960,1972,1985,1997,2010,2022,2035
};
// from 0 to 1  (step 0.001)
const double PowerOf2_16bit [1000] =
{
		1.001	,
		1.001	,
		1.002	,
		1.003	,
		1.003	,
		1.004	,
		1.005	,
		1.006	,
		1.006	,
		1.007	,
		1.008	,
		1.008	,
		1.009	,
		1.010	,
		1.010	,
		1.011	,
		1.012	,
		1.013	,
		1.013	,
		1.014	,
		1.015	,
		1.015	,
		1.016	,
		1.017	,
		1.017	,
		1.018	,
		1.019	,
		1.020	,
		1.020	,
		1.021	,
		1.022	,
		1.022	,
		1.023	,
		1.024	,
		1.025	,
		1.025	,
		1.026	,
		1.027	,
		1.027	,
		1.028	,
		1.029	,
		1.030	,
		1.030	,
		1.031	,
		1.032	,
		1.032	,
		1.033	,
		1.034	,
		1.035	,
		1.035	,
		1.036	,
		1.037	,
		1.037	,
		1.038	,
		1.039	,
		1.040	,
		1.040	,
		1.041	,
		1.042	,
		1.042	,
		1.043	,
		1.044	,
		1.045	,
		1.045	,
		1.046	,
		1.047	,
		1.048	,
		1.048	,
		1.049	,
		1.050	,
		1.050	,
		1.051	,
		1.052	,
		1.053	,
		1.053	,
		1.054	,
		1.055	,
		1.056	,
		1.056	,
		1.057	,
		1.058	,
		1.058	,
		1.059	,
		1.060	,
		1.061	,
		1.061	,
		1.062	,
		1.063	,
		1.064	,
		1.064	,
		1.065	,
		1.066	,
		1.067	,
		1.067	,
		1.068	,
		1.069	,
		1.070	,
		1.070	,
		1.071	,
		1.072	,
		1.073	,
		1.073	,
		1.074	,
		1.075	,
		1.075	,
		1.076	,
		1.077	,
		1.078	,
		1.078	,
		1.079	,
		1.080	,
		1.081	,
		1.081	,
		1.082	,
		1.083	,
		1.084	,
		1.084	,
		1.085	,
		1.086	,
		1.087	,
		1.087	,
		1.088	,
		1.089	,
		1.090	,
		1.091	,
		1.091	,
		1.092	,
		1.093	,
		1.094	,
		1.094	,
		1.095	,
		1.096	,
		1.097	,
		1.097	,
		1.098	,
		1.099	,
		1.100	,
		1.100	,
		1.101	,
		1.102	,
		1.103	,
		1.103	,
		1.104	,
		1.105	,
		1.106	,
		1.106	,
		1.107	,
		1.108	,
		1.109	,
		1.110	,
		1.110	,
		1.111	,
		1.112	,
		1.113	,
		1.113	,
		1.114	,
		1.115	,
		1.116	,
		1.117	,
		1.117	,
		1.118	,
		1.119	,
		1.120	,
		1.120	,
		1.121	,
		1.122	,
		1.123	,
		1.123	,
		1.124	,
		1.125	,
		1.126	,
		1.127	,
		1.127	,
		1.128	,
		1.129	,
		1.130	,
		1.131	,
		1.131	,
		1.132	,
		1.133	,
		1.134	,
		1.134	,
		1.135	,
		1.136	,
		1.137	,
		1.138	,
		1.138	,
		1.139	,
		1.140	,
		1.141	,
		1.142	,
		1.142	,
		1.143	,
		1.144	,
		1.145	,
		1.146	,
		1.146	,
		1.147	,
		1.148	,
		1.149	,
		1.149	,
		1.150	,
		1.151	,
		1.152	,
		1.153	,
		1.153	,
		1.154	,
		1.155	,
		1.156	,
		1.157	,
		1.157	,
		1.158	,
		1.159	,
		1.160	,
		1.161	,
		1.162	,
		1.162	,
		1.163	,
		1.164	,
		1.165	,
		1.166	,
		1.166	,
		1.167	,
		1.168	,
		1.169	,
		1.170	,
		1.170	,
		1.171	,
		1.172	,
		1.173	,
		1.174	,
		1.174	,
		1.175	,
		1.176	,
		1.177	,
		1.178	,
		1.179	,
		1.179	,
		1.180	,
		1.181	,
		1.182	,
		1.183	,
		1.183	,
		1.184	,
		1.185	,
		1.186	,
		1.187	,
		1.188	,
		1.188	,
		1.189	,
		1.190	,
		1.191	,
		1.192	,
		1.193	,
		1.193	,
		1.194	,
		1.195	,
		1.196	,
		1.197	,
		1.197	,
		1.198	,
		1.199	,
		1.200	,
		1.201	,
		1.202	,
		1.202	,
		1.203	,
		1.204	,
		1.205	,
		1.206	,
		1.207	,
		1.207	,
		1.208	,
		1.209	,
		1.210	,
		1.211	,
		1.212	,
		1.213	,
		1.213	,
		1.214	,
		1.215	,
		1.216	,
		1.217	,
		1.218	,
		1.218	,
		1.219	,
		1.220	,
		1.221	,
		1.222	,
		1.223	,
		1.223	,
		1.224	,
		1.225	,
		1.226	,
		1.227	,
		1.228	,
		1.229	,
		1.229	,
		1.230	,
		1.231	,
		1.232	,
		1.233	,
		1.234	,
		1.235	,
		1.235	,
		1.236	,
		1.237	,
		1.238	,
		1.239	,
		1.240	,
		1.241	,
		1.241	,
		1.242	,
		1.243	,
		1.244	,
		1.245	,
		1.246	,
		1.247	,
		1.247	,
		1.248	,
		1.249	,
		1.250	,
		1.251	,
		1.252	,
		1.253	,
		1.254	,
		1.254	,
		1.255	,
		1.256	,
		1.257	,
		1.258	,
		1.259	,
		1.260	,
		1.261	,
		1.261	,
		1.262	,
		1.263	,
		1.264	,
		1.265	,
		1.266	,
		1.267	,
		1.268	,
		1.268	,
		1.269	,
		1.270	,
		1.271	,
		1.272	,
		1.273	,
		1.274	,
		1.275	,
		1.275	,
		1.276	,
		1.277	,
		1.278	,
		1.279	,
		1.280	,
		1.281	,
		1.282	,
		1.283	,
		1.283	,
		1.284	,
		1.285	,
		1.286	,
		1.287	,
		1.288	,
		1.289	,
		1.290	,
		1.291	,
		1.291	,
		1.292	,
		1.293	,
		1.294	,
		1.295	,
		1.296	,
		1.297	,
		1.298	,
		1.299	,
		1.300	,
		1.300	,
		1.301	,
		1.302	,
		1.303	,
		1.304	,
		1.305	,
		1.306	,
		1.307	,
		1.308	,
		1.309	,
		1.309	,
		1.310	,
		1.311	,
		1.312	,
		1.313	,
		1.314	,
		1.315	,
		1.316	,
		1.317	,
		1.318	,
		1.319	,
		1.320	,
		1.320	,
		1.321	,
		1.322	,
		1.323	,
		1.324	,
		1.325	,
		1.326	,
		1.327	,
		1.328	,
		1.329	,
		1.330	,
		1.331	,
		1.331	,
		1.332	,
		1.333	,
		1.334	,
		1.335	,
		1.336	,
		1.337	,
		1.338	,
		1.339	,
		1.340	,
		1.341	,
		1.342	,
		1.343	,
		1.344	,
		1.344	,
		1.345	,
		1.346	,
		1.347	,
		1.348	,
		1.349	,
		1.350	,
		1.351	,
		1.352	,
		1.353	,
		1.354	,
		1.355	,
		1.356	,
		1.357	,
		1.358	,
		1.358	,
		1.359	,
		1.360	,
		1.361	,
		1.362	,
		1.363	,
		1.364	,
		1.365	,
		1.366	,
		1.367	,
		1.368	,
		1.369	,
		1.370	,
		1.371	,
		1.372	,
		1.373	,
		1.374	,
		1.375	,
		1.376	,
		1.376	,
		1.377	,
		1.378	,
		1.379	,
		1.380	,
		1.381	,
		1.382	,
		1.383	,
		1.384	,
		1.385	,
		1.386	,
		1.387	,
		1.388	,
		1.389	,
		1.390	,
		1.391	,
		1.392	,
		1.393	,
		1.394	,
		1.395	,
		1.396	,
		1.397	,
		1.398	,
		1.399	,
		1.400	,
		1.401	,
		1.402	,
		1.402	,
		1.403	,
		1.404	,
		1.405	,
		1.406	,
		1.407	,
		1.408	,
		1.409	,
		1.410	,
		1.411	,
		1.412	,
		1.413	,
		1.414	,
		1.415	,
		1.416	,
		1.417	,
		1.418	,
		1.419	,
		1.420	,
		1.421	,
		1.422	,
		1.423	,
		1.424	,
		1.425	,
		1.426	,
		1.427	,
		1.428	,
		1.429	,
		1.430	,
		1.431	,
		1.432	,
		1.433	,
		1.434	,
		1.435	,
		1.436	,
		1.437	,
		1.438	,
		1.439	,
		1.440	,
		1.441	,
		1.442	,
		1.443	,
		1.444	,
		1.445	,
		1.446	,
		1.447	,
		1.448	,
		1.449	,
		1.450	,
		1.451	,
		1.452	,
		1.453	,
		1.454	,
		1.455	,
		1.456	,
		1.457	,
		1.458	,
		1.459	,
		1.460	,
		1.461	,
		1.462	,
		1.463	,
		1.464	,
		1.465	,
		1.466	,
		1.467	,
		1.468	,
		1.469	,
		1.470	,
		1.471	,
		1.472	,
		1.473	,
		1.474	,
		1.475	,
		1.476	,
		1.477	,
		1.478	,
		1.479	,
		1.480	,
		1.481	,
		1.482	,
		1.483	,
		1.485	,
		1.486	,
		1.487	,
		1.488	,
		1.489	,
		1.490	,
		1.491	,
		1.492	,
		1.493	,
		1.494	,
		1.495	,
		1.496	,
		1.497	,
		1.498	,
		1.499	,
		1.500	,
		1.501	,
		1.502	,
		1.503	,
		1.504	,
		1.505	,
		1.506	,
		1.507	,
		1.508	,
		1.509	,
		1.510	,
		1.512	,
		1.513	,
		1.514	,
		1.515	,
		1.516	,
		1.517	,
		1.518	,
		1.519	,
		1.520	,
		1.521	,
		1.522	,
		1.523	,
		1.524	,
		1.525	,
		1.526	,
		1.527	,
		1.528	,
		1.529	,
		1.530	,
		1.532	,
		1.533	,
		1.534	,
		1.535	,
		1.536	,
		1.537	,
		1.538	,
		1.539	,
		1.540	,
		1.541	,
		1.542	,
		1.543	,
		1.544	,
		1.545	,
		1.546	,
		1.548	,
		1.549	,
		1.550	,
		1.551	,
		1.552	,
		1.553	,
		1.554	,
		1.555	,
		1.556	,
		1.557	,
		1.558	,
		1.559	,
		1.560	,
		1.562	,
		1.563	,
		1.564	,
		1.565	,
		1.566	,
		1.567	,
		1.568	,
		1.569	,
		1.570	,
		1.571	,
		1.572	,
		1.574	,
		1.575	,
		1.576	,
		1.577	,
		1.578	,
		1.579	,
		1.580	,
		1.581	,
		1.582	,
		1.583	,
		1.584	,
		1.586	,
		1.587	,
		1.588	,
		1.589	,
		1.590	,
		1.591	,
		1.592	,
		1.593	,
		1.594	,
		1.595	,
		1.597	,
		1.598	,
		1.599	,
		1.600	,
		1.601	,
		1.602	,
		1.603	,
		1.604	,
		1.605	,
		1.607	,
		1.608	,
		1.609	,
		1.610	,
		1.611	,
		1.612	,
		1.613	,
		1.614	,
		1.616	,
		1.617	,
		1.618	,
		1.619	,
		1.620	,
		1.621	,
		1.622	,
		1.623	,
		1.625	,
		1.626	,
		1.627	,
		1.628	,
		1.629	,
		1.630	,
		1.631	,
		1.632	,
		1.634	,
		1.635	,
		1.636	,
		1.637	,
		1.638	,
		1.639	,
		1.640	,
		1.641	,
		1.643	,
		1.644	,
		1.645	,
		1.646	,
		1.647	,
		1.648	,
		1.649	,
		1.651	,
		1.652	,
		1.653	,
		1.654	,
		1.655	,
		1.656	,
		1.657	,
		1.659	,
		1.660	,
		1.661	,
		1.662	,
		1.663	,
		1.664	,
		1.666	,
		1.667	,
		1.668	,
		1.669	,
		1.670	,
		1.671	,
		1.672	,
		1.674	,
		1.675	,
		1.676	,
		1.677	,
		1.678	,
		1.679	,
		1.681	,
		1.682	,
		1.683	,
		1.684	,
		1.685	,
		1.686	,
		1.688	,
		1.689	,
		1.690	,
		1.691	,
		1.692	,
		1.693	,
		1.695	,
		1.696	,
		1.697	,
		1.698	,
		1.699	,
		1.701	,
		1.702	,
		1.703	,
		1.704	,
		1.705	,
		1.706	,
		1.708	,
		1.709	,
		1.710	,
		1.711	,
		1.712	,
		1.714	,
		1.715	,
		1.716	,
		1.717	,
		1.718	,
		1.720	,
		1.721	,
		1.722	,
		1.723	,
		1.724	,
		1.725	,
		1.727	,
		1.728	,
		1.729	,
		1.730	,
		1.731	,
		1.733	,
		1.734	,
		1.735	,
		1.736	,
		1.737	,
		1.739	,
		1.740	,
		1.741	,
		1.742	,
		1.744	,
		1.745	,
		1.746	,
		1.747	,
		1.748	,
		1.750	,
		1.751	,
		1.752	,
		1.753	,
		1.754	,
		1.756	,
		1.757	,
		1.758	,
		1.759	,
		1.761	,
		1.762	,
		1.763	,
		1.764	,
		1.765	,
		1.767	,
		1.768	,
		1.769	,
		1.770	,
		1.772	,
		1.773	,
		1.774	,
		1.775	,
		1.776	,
		1.778	,
		1.779	,
		1.780	,
		1.781	,
		1.783	,
		1.784	,
		1.785	,
		1.786	,
		1.788	,
		1.789	,
		1.790	,
		1.791	,
		1.793	,
		1.794	,
		1.795	,
		1.796	,
		1.798	,
		1.799	,
		1.800	,
		1.801	,
		1.803	,
		1.804	,
		1.805	,
		1.806	,
		1.808	,
		1.809	,
		1.810	,
		1.811	,
		1.813	,
		1.814	,
		1.815	,
		1.816	,
		1.818	,
		1.819	,
		1.820	,
		1.821	,
		1.823	,
		1.824	,
		1.825	,
		1.826	,
		1.828	,
		1.829	,
		1.830	,
		1.831	,
		1.833	,
		1.834	,
		1.835	,
		1.837	,
		1.838	,
		1.839	,
		1.840	,
		1.842	,
		1.843	,
		1.844	,
		1.845	,
		1.847	,
		1.848	,
		1.849	,
		1.851	,
		1.852	,
		1.853	,
		1.854	,
		1.856	,
		1.857	,
		1.858	,
		1.860	,
		1.861	,
		1.862	,
		1.863	,
		1.865	,
		1.866	,
		1.867	,
		1.869	,
		1.870	,
		1.871	,
		1.873	,
		1.874	,
		1.875	,
		1.876	,
		1.878	,
		1.879	,
		1.880	,
		1.882	,
		1.883	,
		1.884	,
		1.886	,
		1.887	,
		1.888	,
		1.889	,
		1.891	,
		1.892	,
		1.893	,
		1.895	,
		1.896	,
		1.897	,
		1.899	,
		1.900	,
		1.901	,
		1.903	,
		1.904	,
		1.905	,
		1.907	,
		1.908	,
		1.909	,
		1.911	,
		1.912	,
		1.913	,
		1.915	,
		1.916	,
		1.917	,
		1.919	,
		1.920	,
		1.921	,
		1.923	,
		1.924	,
		1.925	,
		1.927	,
		1.928	,
		1.929	,
		1.931	,
		1.932	,
		1.933	,
		1.935	,
		1.936	,
		1.937	,
		1.939	,
		1.940	,
		1.941	,
		1.943	,
		1.944	,
		1.945	,
		1.947	,
		1.948	,
		1.949	,
		1.951	,
		1.952	,
		1.953	,
		1.955	,
		1.956	,
		1.957	,
		1.959	,
		1.960	,
		1.962	,
		1.963	,
		1.964	,
		1.966	,
		1.967	,
		1.968	,
		1.970	,
		1.971	,
		1.972	,
		1.974	,
		1.975	,
		1.977	,
		1.978	,
		1.979	,
		1.981	,
		1.982	,
		1.983	,
		1.985	,
		1.986	,
		1.988	,
		1.989	,
		1.990	,
		1.992	,
		1.993	,
		1.994	,
		1.996	,
		1.997	,
		1.999	,
		2.000	,




};

const uint16_t Triangle1024_12bit [1024] =
{
8,16,24,32,40,48,56,64,
72,80,88,96,104,112,120,128,
136,144,152,160,168,176,184,192,
200,208,216,224,232,240,248,256,
264,272,280,288,296,304,312,320,
328,336,344,352,360,368,376,384,
392,400,408,416,424,432,440,448,
456,464,472,480,488,496,504,512,
520,528,536,544,552,560,568,576,
584,592,600,608,616,624,632,640,
648,656,664,672,680,688,696,704,
712,720,728,736,744,752,760,768,
776,784,792,800,808,816,824,832,
840,848,856,864,872,880,888,896,
904,912,920,928,936,944,952,960,
968,976,984,992,1000,1008,1016,1024,
1032,1040,1048,1056,1064,1072,1080,1088,
1096,1104,1112,1120,1128,1136,1144,1152,
1160,1168,1176,1184,1192,1200,1208,1216,
1224,1232,1240,1248,1256,1264,1272,1280,
1288,1296,1304,1312,1320,1328,1336,1344,
1352,1360,1368,1376,1384,1392,1400,1408,
1416,1424,1432,1440,1448,1456,1464,1472,
1480,1488,1496,1504,1512,1520,1528,1536,
1544,1552,1560,1568,1576,1584,1592,1600,
1608,1616,1624,1632,1640,1648,1656,1664,
1672,1680,1688,1696,1704,1712,1720,1728,
1736,1744,1752,1760,1768,1776,1784,1792,
1800,1808,1816,1824,1832,1840,1848,1856,
1864,1872,1880,1888,1896,1904,1912,1920,
1928,1936,1944,1952,1960,1968,1976,1984,
1992,2000,2008,2016,2024,2032,2040,2048,
2055,2063,2071,2079,2087,2095,2103,2111,
2119,2127,2135,2143,2151,2159,2167,2175,
2183,2191,2199,2207,2215,2223,2231,2239,
2247,2255,2263,2271,2279,2287,2295,2303,
2311,2319,2327,2335,2343,2351,2359,2367,
2375,2383,2391,2399,2407,2415,2423,2431,
2439,2447,2455,2463,2471,2479,2487,2495,
2503,2511,2519,2527,2535,2543,2551,2559,
2567,2575,2583,2591,2599,2607,2615,2623,
2631,2639,2647,2655,2663,2671,2679,2687,
2695,2703,2711,2719,2727,2735,2743,2751,
2759,2767,2775,2783,2791,2799,2807,2815,
2823,2831,2839,2847,2855,2863,2871,2879,
2887,2895,2903,2911,2919,2927,2935,2943,
2951,2959,2967,2975,2983,2991,2999,3007,
3015,3023,3031,3039,3047,3055,3063,3071,
3079,3087,3095,3103,3111,3119,3127,3135,
3143,3151,3159,3167,3175,3183,3191,3199,
3207,3215,3223,3231,3239,3247,3255,3263,
3271,3279,3287,3295,3303,3311,3319,3327,
3335,3343,3351,3359,3367,3375,3383,3391,
3399,3407,3415,3423,3431,3439,3447,3455,
3463,3471,3479,3487,3495,3503,3511,3519,
3527,3535,3543,3551,3559,3567,3575,3583,
3591,3599,3607,3615,3623,3631,3639,3647,
3655,3663,3671,3679,3687,3695,3703,3711,
3719,3727,3735,3743,3751,3759,3767,3775,
3783,3791,3799,3807,3815,3823,3831,3839,
3847,3855,3863,3871,3879,3887,3895,3903,
3911,3919,3927,3935,3943,3951,3959,3967,
3975,3983,3991,3999,4007,4015,4023,4031,
4039,4047,4055,4063,4071,4079,4087,4095,
4087,4079,4071,4063,4055,4047,4039,4031,
4023,4015,4007,3999,3991,3983,3975,3967,
3959,3951,3943,3935,3927,3919,3911,3903,
3895,3887,3879,3871,3863,3855,3847,3839,
3831,3823,3815,3807,3799,3791,3783,3775,
3767,3759,3751,3743,3735,3727,3719,3711,
3703,3695,3687,3679,3671,3663,3655,3647,
3639,3631,3623,3615,3607,3599,3591,3583,
3575,3567,3559,3551,3543,3535,3527,3519,
3511,3503,3495,3487,3479,3471,3463,3455,
3447,3439,3431,3423,3415,3407,3399,3391,
3383,3375,3367,3359,3351,3343,3335,3327,
3319,3311,3303,3295,3287,3279,3271,3263,
3255,3247,3239,3231,3223,3215,3207,3199,
3191,3183,3175,3167,3159,3151,3143,3135,
3127,3119,3111,3103,3095,3087,3079,3071,
3063,3055,3047,3039,3031,3023,3015,3007,
2999,2991,2983,2975,2967,2959,2951,2943,
2935,2927,2919,2911,2903,2895,2887,2879,
2871,2863,2855,2847,2839,2831,2823,2815,
2807,2799,2791,2783,2775,2767,2759,2751,
2743,2735,2727,2719,2711,2703,2695,2687,
2679,2671,2663,2655,2647,2639,2631,2623,
2615,2607,2599,2591,2583,2575,2567,2559,
2551,2543,2535,2527,2519,2511,2503,2495,
2487,2479,2471,2463,2455,2447,2439,2431,
2423,2415,2407,2399,2391,2383,2375,2367,
2359,2351,2343,2335,2327,2319,2311,2303,
2295,2287,2279,2271,2263,2255,2247,2239,
2231,2223,2215,2207,2199,2191,2183,2175,
2167,2159,2151,2143,2135,2127,2119,2111,
2103,2095,2087,2079,2071,2063,2055,2048,
2040,2032,2024,2016,2008,2000,1992,1984,
1976,1968,1960,1952,1944,1936,1928,1920,
1912,1904,1896,1888,1880,1872,1864,1856,
1848,1840,1832,1824,1816,1808,1800,1792,
1784,1776,1768,1760,1752,1744,1736,1728,
1720,1712,1704,1696,1688,1680,1672,1664,
1656,1648,1640,1632,1624,1616,1608,1600,
1592,1584,1576,1568,1560,1552,1544,1536,
1528,1520,1512,1504,1496,1488,1480,1472,
1464,1456,1448,1440,1432,1424,1416,1408,
1400,1392,1384,1376,1368,1360,1352,1344,
1336,1328,1320,1312,1304,1296,1288,1280,
1272,1264,1256,1248,1240,1232,1224,1216,
1208,1200,1192,1184,1176,1168,1160,1152,
1144,1136,1128,1120,1112,1104,1096,1088,
1080,1072,1064,1056,1048,1040,1032,1024,
1016,1008,1000,992,984,976,968,960,
952,944,936,928,920,912,904,896,
888,880,872,864,856,848,840,832,
824,816,808,800,792,784,776,768,
760,752,744,736,728,720,712,704,
696,688,680,672,664,656,648,640,
632,624,616,608,600,592,584,576,
568,560,552,544,536,528,520,512,
504,496,488,480,472,464,456,448,
440,432,424,416,408,400,392,384,
376,368,360,352,344,336,328,320,
312,304,296,288,280,272,264,256,
248,240,232,224,216,208,200,192,
184,176,168,160,152,144,136,128,
120,112,104,96,88,80,72,64,
56,48,40,32,24,16,8,0
};
const uint16_t Sine512_12bit [] =
{
2048,2073,2098,2123,2148,2173,2198,2223,
2248,2273,2298,2323,2348,2373,2398,2422,
2447,2472,2496,2521,2545,2569,2594,2618,
2642,2666,2690,2714,2737,2761,2784,2808,
2831,2854,2877,2900,2923,2946,2968,2990,
3013,3035,3057,3078,3100,3122,3143,3164,
3185,3206,3226,3247,3267,3287,3307,3327,
3346,3366,3385,3404,3423,3441,3459,3477,
3495,3513,3530,3548,3565,3581,3598,3614,
3630,3646,3662,3677,3692,3707,3722,3736,
3750,3764,3777,3791,3804,3816,3829,3841,
3853,3865,3876,3888,3898,3909,3919,3929,
3939,3949,3958,3967,3975,3984,3992,3999,
4007,4014,4021,4027,4034,4040,4045,4051,
4056,4060,4065,4069,4073,4076,4080,4083,
4085,4087,4089,4091,4093,4094,4094,4095,
4095,4095,4094,4094,4093,4091,4089,4087,
4085,4083,4080,4076,4073,4069,4065,4060,
4056,4051,4045,4040,4034,4027,4021,4014,
4007,3999,3992,3984,3975,3967,3958,3949,
3939,3929,3919,3909,3898,3888,3876,3865,
3853,3841,3829,3816,3804,3791,3777,3764,
3750,3736,3722,3707,3692,3677,3662,3646,
3630,3614,3598,3581,3565,3548,3530,3513,
3495,3477,3459,3441,3423,3404,3385,3366,
3346,3327,3307,3287,3267,3247,3226,3206,
3185,3164,3143,3122,3100,3078,3057,3035,
3013,2990,2968,2946,2923,2900,2877,2854,
2831,2808,2784,2761,2737,2714,2690,2666,
2642,2618,2594,2569,2545,2521,2496,2472,
2447,2422,2398,2373,2348,2323,2298,2273,
2248,2223,2198,2173,2148,2123,2098,2073,
2048,2022,1997,1972,1947,1922,1897,1872,
1847,1822,1797,1772,1747,1722,1697,1673,
1648,1623,1599,1574,1550,1526,1501,1477,
1453,1429,1405,1381,1358,1334,1311,1287,
1264,1241,1218,1195,1172,1149,1127,1105,
1082,1060,1038,1017,995,973,952,931,
910,889,869,848,828,808,788,768,
749,729,710,691,672,654,636,618,
600,582,565,547,530,514,497,481,
465,449,433,418,403,388,373,359,
345,331,318,304,291,279,266,254,
242,230,219,207,197,186,176,166,
156,146,137,128,120,111,103,96,
88,81,74,68,61,55,50,44,
39,35,30,26,22,19,15,12,
10,8,6,4,2,1,1,0,
0,0,1,1,2,4,6,8,
10,12,15,19,22,26,30,35,
39,44,50,55,61,68,74,81,
88,96,103,111,120,128,137,146,
156,166,176,186,197,207,219,230,
242,254,266,279,291,304,318,331,
345,359,373,388,403,418,433,449,
465,481,497,514,530,547,565,582,
600,618,636,654,672,691,710,729,
749,768,788,808,828,848,869,889,
910,931,952,973,995,1017,1038,1060,
1082,1105,1127,1149,1172,1195,1218,1241,
1264,1287,1311,1334,1358,1381,1405,1429,
1453,1477,1501,1526,1550,1574,1599,1623,
1648,1673,1697,1722,1747,1772,1797,1822,
1847,1872,1897,1922,1947,1972,1997,2022
};

const uint16_t Sine256_12bit [] =
{
		2048,2098,2148,2198,2248,2298,2348,2398,
		2447,2496,2545,2594,2642,2690,2737,2784,
		2831,2877,2923,2968,3013,3057,3100,3143,
		3185,3226,3267,3307,3346,3385,3423,3459,
		3495,3530,3565,3598,3630,3662,3692,3722,
		3750,3777,3804,3829,3853,3876,3898,3919,
		3939,3958,3975,3992,4007,4021,4034,4045,
		4056,4065,4073,4080,4085,4089,4093,4094,
		4095,4094,4093,4089,4085,4080,4073,4065,
		4056,4045,4034,4021,4007,3992,3975,3958,
		3939,3919,3898,3876,3853,3829,3804,3777,
		3750,3722,3692,3662,3630,3598,3565,3530,
		3495,3459,3423,3385,3346,3307,3267,3226,
		3185,3143,3100,3057,3013,2968,2923,2877,
		2831,2784,2737,2690,2642,2594,2545,2496,
		2447,2398,2348,2298,2248,2198,2148,2098,
		2048,1997,1947,1897,1847,1797,1747,1697,
		1648,1599,1550,1501,1453,1405,1358,1311,
		1264,1218,1172,1127,1082,1038,995,952,
		910,869,828,788,749,710,672,636,
		600,565,530,497,465,433,403,373,
		345,318,291,266,242,219,197,176,
		156,137,120,103,88,74,61,50,
		39,30,22,15,10,6,2,1,
		0,1,2,6,10,15,22,30,
		39,50,61,74,88,103,120,137,
		156,176,197,219,242,266,291,318,
		345,373,403,433,465,497,530,565,
		600,636,672,710,749,788,828,869,
		910,952,995,1038,1082,1127,1172,1218,
		1264,1311,1358,1405,1453,1501,1550,1599,
		1648,1697,1747,1797,1847,1897,1947,1997

};

