#ifndef SRC_CONSTANTS_H
#define SRC_CONSTANTS_H

namespace Constants {
	//DriveTrain
	static constexpr int driveLeftMasterID  = 12;
	static constexpr int driveRightMasterID = 14;
	static constexpr int driveLeftSlaveID = 13;
	static constexpr int driveRightSlaveID = 15;

	//Joystick axis and buttons
	static constexpr int driveJoystickChannel = 0;
	static constexpr int driveLeftStickX = 0;
	static constexpr int driveLeftStickY = 1;
	static constexpr int driveL2 = 3;
	static constexpr int driveRightStickY = 5;
	static constexpr int calibrateButton = 12;
	static constexpr int shootButton = 1;
	static constexpr int prepareToShootButton = 3;
	static constexpr int ejectButton = 14;
	static constexpr int stopShooterWheels = 13;
	static constexpr int xButton = 2;

	//Shooter
	static constexpr int shooterLeftTalonID = 8;
	static constexpr int shooterRightTalonID = 9;
	static constexpr int shooterAimTalonID = 10;
	static constexpr int shooterIRPin = 6;
	static constexpr int servoPin = 0;
	static constexpr float shooter100Velocity = 12.33; //via physics major
	static constexpr float shooter75Velocity = 9.2475; //may need to be changed. Currently assuming percent voltage to velocity is linear
	static constexpr float shooter50Velocity = 6.165; //may need to be changed. Currently assuming percent voltage to velocity is linear
	static constexpr float minimumAngle = 32;	//Actual value: needs more accuracy
	static constexpr float maximumAngle = 70; //TODO: temporary value
	static constexpr float servoMaxPosition = 0.9;
	static constexpr float servoMinPosition = 0.3;
	static constexpr float aimDegreesToPotFactor = 545 / 68.2;

	//Arm
	static constexpr int armTalonPin  = 11;

	//Position
	static constexpr float towerX = 0;
	static constexpr float towerY = 0;
	static constexpr float towerHeight = 0;
	static constexpr float xStartPos = 0;
	static constexpr float yStartPos = 0;
	static constexpr float gyroOffset = 90;
	static constexpr int ticksPerRotation = 2048;
	static constexpr float quadratureEncoderFactor = .25;
	static constexpr float wheelRadius = 3 * .0254; //meters

	static constexpr float distances[] = {
			6.4879885,
			6.3198907,
			6.2004894,
			6.1032548,
			6.0194787,
			5.9449957,
			5.8774284,
			5.8152674,
			5.7574837,
			5.7033383,
			5.6522793,
			5.6038815,
			5.5578091,
			5.5137914,
			5.4716066,
			5.4310698,
			5.3920253,
			5.3543404,
			5.3179011,
			5.2826081,
			5.2483748,
			5.2151250,
			5.1827911,
			5.1513127,
			5.1206361,
			5.0907126,
			5.0614984,
			5.0329537,
			5.0050424,
			4.9777312,
			4.9509900,
			4.9247908,
			4.8991079,
			4.8739176,
			4.8491978,
			4.8249282,
			4.8010898,
			4.7776648,
			4.7546367,
			4.7319900,
			4.7097101,
			4.6877835,
			4.6661974,
			4.6449396,
			4.6239987,
			4.6033641,
			4.5830255,
			4.5629734,
			4.5431985,
			4.5236923,
			4.5044465,
			4.4854532,
			4.4667052,
			4.4481952,
			4.4299164,
			4.4118625,
			4.3940273,
			4.3764048,
			4.3589895,
			4.3417759,
			4.3247589,
			4.3079334,
			4.2912947,
			4.2748383,
			4.2585598,
			4.2424549,
			4.2265195,
			4.2107498,
			4.1951419,
			4.1796923,
			4.1643975,
			4.1492540,
			4.1342586,
			4.1194081,
			4.1046996,
			4.0901300,
			4.0756966,
			4.0613965,
			4.0472272,
			4.0331859,
			4.0192703,
			4.0054780,
			3.9918065,
			3.9782536,
			3.9648170,
			3.9514948,
			3.9382847,
			3.9251847,
			3.9121929,
			3.8993074,
			3.8865264,
			3.8738479,
			3.8612703,
			3.8487919,
			3.8364110,
			3.8241260,
			3.8119353,
			3.7998374,
			3.7878308,
			3.7759140,
			3.7640856,
			3.7523443,
			3.7406886,
			3.7291173,
			3.7176291,
			3.7062228,
			3.6948970,
			3.6836506,
			3.6724825,
			3.6613914,
			3.6503764,
			3.6394362,
			3.6285699,
			3.6177764,
			3.6070546,
			3.5964036,
			3.5858224,
			3.5753101,
			3.5648657,
			3.5544883,
			3.5441769,
			3.5339309,
			3.5237492,
			3.5136310,
			3.5035756,
			3.4935821,
			3.4836497,
			3.4737776,
			3.4639651,
			3.4542115,
			3.4445160,
			3.4348779,
			3.4252965,
			3.4157711,
			3.4063010,
			3.3968856,
			3.3875243,
			3.3782163,
			3.3689610,
			3.3597580,
			3.3506064,
			3.3415058,
			3.3324556,
			3.3234551,
			3.3145039,
			3.3056014,
			3.2967470,
			3.2879402,
			3.2791804,
			3.2704672,
			3.2618001,
			3.2531786,
			3.2446021,
			3.2360701,
			3.2275823,
			3.2191382,
			3.2107372,
			3.2023789,
			3.1940630,
			3.1857889,
			3.1775562,
			3.1693645,
			3.1612134,
			3.1531025,
			3.1450313,
			3.1369995,
			3.1290067,
			3.1210525,
			3.1131366,
			3.1052584,
			3.0974178,
			3.0896142,
			3.0818474,
			3.0741170,
			3.0664227,
			3.0587640,
			3.0511408,
			3.0435526,
			3.0359991,
			3.0284800,
			3.0209950,
			3.0135437,
			3.0061260,
			2.9987413,
			2.9913895,
			2.9840703,
			2.9767833,
			2.9695283,
			2.9623049,
			2.9551130,
			2.9479522,
			2.9408223,
			2.9337229,
			2.9266539,
			2.9196150,
			2.9126058,
			2.9056262,
			2.8986758,
			2.8917545,
			2.8848620,
			2.8779981,
			2.8711624,
			2.8643549,
			2.8575752,
			2.8508231,
			2.8440983,
			2.8374008,
			2.8307302,
			2.8240863,
			2.8174689,
			2.8108777,
			2.8043127,
			2.7977735,
			2.7912600,
			2.7847720,
			2.7783092,
			2.7718714,
			2.7654585,
			2.7590703,
			2.7527065,
			2.7463671,
			2.7400517,
			2.7337602,
			2.7274925,
			2.7212483,
			2.7150274,
			2.7088298,
			2.7026551,
			2.6965033,
			2.6903742,
			2.6842676,
			2.6781833,
			2.6721211,
			2.6660810,
			2.6600627,
			2.6540661,
			2.6480910,
			2.6421372,
			2.6362047,
			2.6302932,
			2.6244027,
			2.6185328,
			2.6126836,
			2.6068549,
			2.6010464,
			2.5952582,
			2.5894899,
			2.5837416,
			2.5780130,
			2.5723040,
			2.5666144,
			2.5609442,
			2.5552932,
			2.5496613,
			2.5440483,
			2.5384542,
			2.5328787,
			2.5273217,
			2.5217832,
			2.5162630,
			2.5107610,
			2.5052770,
			2.4998110,
			2.4943628,
			2.4889323,
			2.4835194,
			2.4781239,
			2.4727458,
			2.4673850,
			2.4620412,
			2.4567145,
			2.4514047,
			2.4461116,
			2.4408353,
			2.4355755,
			2.4303322,
			2.4251053,
			2.4198946,
			2.4147001,
			2.4095216,
			2.4043591,
			2.3992124,
			2.3940815,
			2.3889662,
			2.3838665,
			2.3787822,
			2.3737134,
			2.3686597,
			2.3636213,
			2.3585979,
			2.3535895,
			2.3485960,
			2.3436173,
			2.3386533,
			2.3337040,
			2.3287691,
			2.3238487,
			2.3189427,
			2.3140509,
			2.3091734,
			2.3043099,
			2.2994605,
			2.2946249,
			2.2898033,
			2.2849954,
			2.2802012,
			2.2754206,
			2.2706535,
			2.2658998,
			2.2611596,
			2.2564326,
			2.2517189,
			2.2470182,
			2.2423307,
			2.2376561,
			2.2329944,
			2.2283456,
			2.2237096,
			2.2190862,
			2.2144754,
			2.2098772,
			2.2052915,
			2.2007182,
			2.1961572,
			2.1916085,
			2.1870719,
			2.1825475,
			2.1780352,
			2.1735349,
			2.1690465,
			2.1645699,
			2.1601051,
			2.1556521,
			2.1512107,
			2.1467810,
			2.1423627,
			2.1379560,
			2.1335607,
			2.1291767,
			2.1248040,
			2.1204425,
			2.1160922,
			2.1117530,
			2.1074248,
			2.1031077,
			2.0988014,
			2.0945061,
			2.0902215,
			2.0859477,
			2.0816846,
			2.0774321,
			2.0731903,
			2.0689589,
			2.0647380,
			2.0605276,
			2.0563275,
			2.0521377,
			2.0479582,
			2.0437889,
			2.0396297,
			2.0354806,
			2.0313416,
			2.0272126,
			2.0230935,
			2.0189843,
			2.0148849,
			2.0107953,
			2.0067155,
			2.0026453,
			1.9985848,
			1.9945339,
			1.9904926,
			1.9864607,
			1.9824383,
			1.9784252,
			1.9744215,
			1.9704272,
			1.9664421,
			1.9624662,
			1.9584994,
			1.9545418,
			1.9505933,
			1.9466538,
			1.9427233,
			1.9388017,
			1.9348890,
			1.9309852,
			1.9270902,
			1.9232040,
			1.9193264,
			1.9154576,
			1.9115974,
			1.9077458,
			1.9039028,
			1.9000682,
			1.8962422,
			1.8924246,
			1.8886153,
			1.8848145,
			1.8810219,
			1.8772376,
			1.8734616,
			1.8696937,
			1.8659340,
			1.8621824,
			1.8584389,
			1.8547034,
			1.8509759,
			1.8472564,
			1.8435448,
			1.8398411,
			1.8361453,
			1.8324572,
			1.8287770,
			1.8251045,
			1.8214396,
			1.8177825,
			1.8141330,
			1.8104911,
			1.8068568
	};
};

#endif
