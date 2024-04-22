/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#define MAX_TEMP_RANGE         402

/* Define a 2D array to store the ADC values and corresponding temperatures */
const float tempData[][2] = {
    {918, -40.0},
    {919, -39.6},
    {920, -39.2},
    {921, -38.8},
    {922, -38.4},
    {923, -38.0},
    {924, -37.6},
    {925, -37.2},
    {926, -36.8},
    {927, -36.4},
    {928, -36.0},
    {929, -35.6},
    {930, -35.2},
    {931, -34.8},
    {932, -34.4},
    {933, -34.0},
    {934, -33.6},
    {935, -33.2},
    {936, -32.8},
    {937, -32.4},
    {938, -32.0},
    {939, -31.6},
    {940, -31.2},
    {941, -30.8},
    {942, -30.4},
    {943, -30.0},
    {944, -29.8},
    {945, -29.7},
    {946, -29.5},
    {947, -29.3},
    {948, -29.2},
    {949, -29.0},
    {950, -28.8},
    {951, -28.7},
    {952, -28.5},
    {953, -28.3},
    {954, -28.2},
    {955, -28.0},
    {956, -27.8},
    {957, -27.7},
    {958, -27.5},
    {959, -27.3},
    {960, -27.2},
    {961, -27.0},
    {962, -26.8},
    {963, -26.7},
    {964, -26.5},
    {965, -26.3},
    {966, -26.2},
    {967, -26.0},
    {968, -25.8},
    {969, -25.7},
    {970, -25.5},
    {971, -25.3},
    {972, -25.2},
    {973, -25.0},
    {974, -24.8},
    {975, -24.7},
    {976, -24.5},
    {977, -24.3},
    {978, -24.2},
    {979, -24.0},
    {980, -23.8},
    {981, -23.7},
    {982, -23.5},
    {983, -23.3},
    {984, -23.2},
    {985, -23.0},
    {986, -22.8},
    {987, -22.7},
    {988, -22.5},
    {989, -22.3},
    {990, -22.2},
    {991, -22.0},
    {992, -21.8},
    {993, -21.7},
    {994, -21.5},
    {995, -21.3},
    {996, -21.2},
    {997, -21.0},
    {998, -20.8},
    {999, -20.7},
    {1000, -20.5},
    {1001, -20.3},
    {1002, -20.2},
    {1003, -20.0},
    {1004, -19.2},
    {1005, -18.5},
    {1006, -17.7},
    {1007, -16.9},
    {1008, -16.2},
    {1009, -15.4},
    {1010, -14.6},
    {1011, -13.8},
    {1012, -13.1},
    {1013, -12.3},
    {1014, -11.5},
    {1015, -10.8},
    {1016, -10.0},
    {1017, -9.4},
    {1018, -8.8},
    {1019, -8.1},
    {1020, -7.5},
    {1021, -6.9},
    {1022, -6.3},
    {1023, -5.6},
    {1024, -5.0},
    {1025, -4.4},
    {1026, -3.8},
    {1027, -3.1},
    {1028, -2.5},
    {1029, -1.9},
    {1030, -1.3},
    {1031, -0.6},
    {1032, 0.0},
    {1033, 0.7},
    {1034, 1.3},
    {1035, 2.0},
    {1036, 2.7},
    {1037, 3.3},
    {1038, 4.0},
    {1039, 4.7},
    {1040, 5.3},
    {1041, 6.0},
    {1042, 6.7},
    {1043, 7.3},
    {1044, 8.0},
    {1045, 8.7},
    {1046, 9.3},
    {1047, 10.0},
    {1048, 10.4},
    {1049, 10.9},
    {1050, 11.3},
    {1051, 11.7},
    {1052, 12.2},
    {1053, 12.6},
    {1054, 13.0},
    {1055, 13.5},
    {1056, 13.9},
    {1057, 14.3},
    {1058, 14.8},
    {1059, 15.2},
    {1060, 15.7},
    {1061, 16.1},
    {1062, 16.5},
    {1063, 17.0},
    {1064, 17.4},
    {1065, 17.8},
    {1066, 18.3},
    {1067, 18.7},
    {1068, 19.1},
    {1069, 19.6},
    {1070, 20.0},
    {1071, 20.2},
    {1072, 20.3},
    {1073, 20.5},
    {1074, 20.7},
    {1075, 20.9},
    {1076, 21.0},
    {1077, 21.2},
    {1078, 21.4},
    {1079, 21.6},
    {1080, 21.7},
    {1081, 21.9},
    {1082, 22.1},
    {1083, 22.2},
    {1084, 22.4},
    {1085, 22.6},
    {1086, 22.8},
    {1087, 22.9},
    {1088, 23.1},
    {1089, 23.3},
    {1090, 23.4},
    {1091, 23.6},
    {1092, 23.8},
    {1093, 24.0},
    {1094, 24.1},
    {1095, 24.3},
    {1096, 24.5},
    {1097, 24.7},
    {1098, 24.8},
    {1099, 25.0},
    {1100, 25.2},
    {1101, 25.3},
    {1102, 25.5},
    {1103, 25.7},
    {1104, 25.9},
    {1105, 26.0},
    {1106, 26.2},
    {1107, 26.4},
    {1108, 26.6},
    {1109, 26.7},
    {1110, 26.9},
    {1111, 27.1},
    {1112, 27.2},
    {1113, 27.4},
    {1114, 27.6},
    {1115, 27.8},
    {1116, 27.9},
    {1117, 28.1},
    {1118, 28.3},
    {1119, 28.4},
    {1120, 28.6},
    {1121, 28.8},
    {1122, 29.0},
    {1123, 29.1},
    {1124, 29.3},
    {1125, 29.5},
    {1126, 29.7},
    {1127, 29.8},
    {1128, 30.0},
    {1129, 30.3},
    {1130, 30.5},
    {1131, 30.8},
    {1132, 31.0},
    {1133, 31.3},
    {1134, 31.5},
    {1135, 31.8},
    {1136, 32.0},
    {1137, 32.3},
    {1138, 32.5},
    {1139, 32.8},
    {1140, 33.0},
    {1141, 33.3},
    {1142, 33.5},
    {1143, 33.8},
    {1144, 34.0},
    {1145, 34.3},
    {1146, 34.5},
    {1147, 34.8},
    {1148, 35.0},
    {1149, 35.3},
    {1150, 35.5},
    {1151, 35.8},
    {1152, 36.0},
    {1153, 36.3},
    {1154, 36.5},
    {1155, 36.8},
    {1156, 37.0},
    {1157, 37.3},
    {1158, 37.5},
    {1159, 37.8},
    {1160, 38.0},
    {1161, 38.3},
    {1162, 38.5},
    {1163, 38.8},
    {1164, 39.0},
    {1165, 39.3},
    {1166, 39.5},
    {1167, 39.8},
    {1168, 40.0},
    {1169, 40.5},
    {1170, 40.9},
    {1171, 41.4},
    {1172, 41.8},
    {1173, 42.3},
    {1174, 42.7},
    {1175, 43.2},
    {1176, 43.6},
    {1177, 44.1},
    {1178, 44.5},
    {1179, 45.0},
    {1180, 45.5},
    {1181, 45.9},
    {1182, 46.4},
    {1183, 46.8},
    {1184, 47.3},
    {1185, 47.7},
    {1186, 48.2},
    {1187, 48.6},
    {1188, 49.1},
    {1189, 49.5},
    {1190, 50.0},
    {1191, 50.2},
    {1192, 50.3},
    {1193, 50.5},
    {1194, 50.7},
    {1195, 50.9},
    {1196, 51.0},
    {1197, 51.2},
    {1198, 51.4},
    {1199, 51.6},
    {1200, 51.7},
    {1201, 51.9},
    {1202, 52.1},
    {1203, 52.2},
    {1204, 52.4},
    {1205, 52.6},
    {1206, 52.8},
    {1207, 52.9},
    {1208, 53.1},
    {1209, 53.3},
    {1210, 53.4},
    {1211, 53.6},
    {1212, 53.8},
    {1213, 54.0},
    {1214, 54.1},
    {1215, 54.3},
    {1216, 54.5},
    {1217, 54.7},
    {1218, 54.8},
    {1219, 55.0},
    {1220, 55.2},
    {1221, 55.3},
    {1222, 55.5},
    {1223, 55.7},
    {1224, 55.9},
    {1225, 56.0},
    {1226, 56.2},
    {1227, 56.4},
    {1228, 56.6},
    {1229, 56.7},
    {1230, 56.9},
    {1231, 57.1},
    {1232, 57.2},
    {1233, 57.4},
    {1234, 57.6},
    {1235, 57.8},
    {1236, 57.9},
    {1237, 58.1},
    {1238, 58.3},
    {1239, 58.4},
    {1240, 58.6},
    {1241, 58.8},
    {1242, 59.0},
    {1243, 59.1},
    {1244, 59.3},
    {1245, 59.5},
    {1246, 59.7},
    {1247, 59.8},
    {1248, 60.0},
    {1249, 60.3},
    {1250, 60.7},
    {1251, 61.0},
    {1252, 61.3},
    {1253, 61.7},
    {1254, 62.0},
    {1255, 62.3},
    {1256, 62.7},
    {1257, 63.0},
    {1258, 63.3},
    {1259, 63.7},
    {1260, 64.0},
    {1261, 64.3},
    {1262, 64.7},
    {1263, 65.0},
    {1264, 65.3},
    {1265, 65.7},
    {1266, 66.0},
    {1267, 66.3},
    {1268, 66.7},
    {1269, 67.0},
    {1270, 67.3},
    {1271, 67.7},
    {1272, 68.0},
    {1273, 68.3},
    {1274, 68.7},
    {1275, 69.0},
    {1276, 69.3},
    {1277, 69.7},
    {1278, 70.0},
    {1279, 70.4},
    {1280, 70.9},
    {1281, 71.3},
    {1282, 71.7},
    {1283, 72.2},
    {1284, 72.6},
    {1285, 73.0},
    {1286, 73.5},
    {1287, 73.9},
    {1288, 74.3},
    {1289, 74.8},
    {1290, 75.2},
    {1291, 75.7},
    {1292, 76.1},
    {1293, 76.5},
    {1294, 77.0},
    {1295, 77.4},
    {1296, 77.8},
    {1297, 78.3},
    {1298, 78.7},
    {1299, 79.1},
    {1300, 79.6},
    {1301, 80.0},
    {1302, 80.5},
    {1303, 81.1},
    {1304, 81.6},
    {1305, 82.1},
    {1306, 82.6},
    {1307, 83.2},
    {1308, 83.7},
    {1309, 84.2},
    {1310, 84.7},
    {1311, 85.3},
    {1312, 85.8},
    {1313, 86.3},
    {1314, 86.8},
    {1315, 87.4},
    {1316, 87.9},
    {1317, 88.4},
    {1318, 88.9},
    {1319, 89.5},
    {1320, 90.0}
};

/*
 * @func         : float get_temperature (uint32_t adc_value)
 * @brief        : Converts the digital value into a temperature reading
 * @parameter[1] : adc_value  : The digital value obtained from TSENS conversion
 * @return       : temperature
 */
float get_temperature (uint32_t adc_value)
{
    int i;

    /* check for temperature operating range */
    if (adc_value < tempData[0][0] || (adc_value > tempData[MAX_TEMP_RANGE][0]))
    {
        return -1;
    }

    for (i = 0; i < sizeof(tempData) / sizeof(tempData[0]); i++)
    {
        /* check if value matches with tempdata */
        if (adc_value == tempData[i][0])
        {
            break;
        }
    }

    return (tempData[i][1]);
}

#endif /* TEMPERATURE_H_ */