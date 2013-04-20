package com.example.autopilot;

public class Aircraft {
	public static float [] OmegaBodyFromEulerRates(float [] EulerRates, float [] EulerAngles){
		float [] OmegaBody = new float [3];

		float phi = EulerAngles[1];
		float theta = EulerAngles[2];

		float [][] Tmat = {{1 , 0 , 0 },
				{0 , (float) Math.cos(phi) , (float) (Math.cos(theta)*Math.sin(phi))},
				{0 , (float) -Math.sin(phi) , (float) (Math.cos(theta)*Math.cos(phi))}};

		OmegaBody = Matrix.multiply(Tmat,EulerRates);

		return OmegaBody;
	}

	public static float [] EulerRatesFromOmegaBody(float[] OmegaBody, float[] EulerAngles) {
		float [] EulerRates = new float [3];
		float phi = EulerAngles[1];
		float theta = EulerAngles[2];

		float[][] Tmat = {{1, (float) (Math.sin(phi)*Math.tan(theta)),(float) (Math.cos(phi)*Math.tan(theta)) },
				{0, (float) Math.cos(phi), (float) -Math.sin(phi)},
				{0, (float) (Math.sin(phi)*1/Math.cos(theta)), (float) (Math.cos(phi)*1/Math.cos(theta))}};

		EulerRates = Matrix.multiply(Tmat,EulerRates);

		return EulerRates;
	}

	public static float [][] Rotation321(float[] EulerAngles) {
		float phi = EulerAngles[0];
		float theta = EulerAngles[1];
		float psi = EulerAngles[2];

		float [][] S = new float[3][3];
		
		S = Matrix.multiply(Matrix.multiply(RotationN(phi,1), RotationN(theta,2)),RotationN(psi,3));
		return S;
	}
	
	public static float [][] RotationN(float phi, int N) {
		float[][] RN = new float [3][3];
		float CPHI = (float) Math.cos(phi);
		float SPHI = (float) Math.sin(phi);
		
		switch (N) {
		case 1:
			RN[0][0] = 1; 		RN[0][1] = 0; 		RN[0][2] = 0;
			RN[1][0] = 0; 		RN[1][1] = CPHI;	RN[1][2] = SPHI;					
			RN[2][0] = 0;		RN[2][1] = -SPHI;	RN[2][2] = CPHI;
		case 2:
			RN[0][0] = CPHI; 	RN[0][1] = 0; 		RN[0][2] = -SPHI;
			RN[1][0] = 0; 		RN[1][1] = 1;		RN[1][2] = 0;					
			RN[2][0] = SPHI;	RN[2][1] = 0;		RN[2][2] = CPHI;
		case 3:
			RN[0][0] = CPHI; 	RN[0][1] = SPHI; 	RN[0][2] = 0;
			RN[1][0] = -SPHI; 	RN[1][1] = CPHI;	RN[1][2] = 0;					
			RN[2][0] = 0;		RN[2][1] = 0;		RN[2][2] = 1;
		}
		return RN;
	}

	public static float [] TransformFromBodyToInertial( float[] vector_body, float[] EulerAngles) {
		return Matrix.multiply(Matrix.transpose(Rotation321(EulerAngles)),vector_body);
	}

	public static float [] TransformFromInertialToBody( float[] vector_inertial, float[] EulerAngles) {
		return Matrix.multiply(Rotation321(EulerAngles), vector_inertial);
	}

	public static float [] VelocityVectorToWindAngles(float[] velocity_body) {
		float speed = Matrix.norm(velocity_body);
		float alpha = (float) Math.atan(velocity_body[3]/velocity_body[1]);
		float beta = (float) Math.asin(velocity_body[2]/speed);

		float [] WindAngles = {speed,alpha,beta};

		return WindAngles;
	} 

	public static final float m = 546;
	public static final float altitude = 15000; //// ft
	public static final float M = 0.6f; // Mach #
	public static final float u0 = 634; // ft/s
	public static final float Q = 301; // lb/ft2 dynamic pressure
	public static final float W = 17578f; // lbs
	public static final float Ix = 8010f; //slug*ft2
	public static final float Iy = 25900f; // slug*ft2
	public static final float Iz = 29280f; // slug*ft2
	public static final float Ixz = 41f; // slug*ft2
	public static final float alpha = 3.4f; // deg
	//// Trim Conditions
	public static final float X0 = 0f;
	public static final float Y0 = 0f;
	public  final float Z0 = -Aircraft.W;
	public static final float L0 = 0f;
	public static final float M0 = 0f;
	public static final float N0 = 0f;

	//// Table B.1b Longitudinal Dimensional Derivatives
	public static final float h0 = 15000f; //[ft]
	public static final float Xu = -0.0129f;    //[1/s]
	public static final float Xalpha = -3.721f; //[ft/2^2]
	public static final float Zu = -0.104f;     //[1/s]
	public static final float Zalpha = -518.9f;  //[ft/s^2]
	public static final float Zalphadot = 0f;
	public static final float Mu = 0.0004f;       //[-/ft*s]
	public static final float Malpha = -12.97f; //[1/s^2]
	public static final float Malphadot = -0.353f; //[1/s]
	public static final float Zq = 0f;
	public static final float Mq = -1.071f;     //[1/s]
	public static final float Xde = 4.02f;    //[ft/s^2]
	public static final float Zde = -57.02f; //[ft/s^2]
	public static final float Mde = -19.46f; //[1/s^2]

	//// Table B.1c Lateral-directional Dimensional Derivatives
	public static final float Ybeta = -144.6f; // ft/s2
	public static final float Lbeta = -35.0f; // 1/s2
	public static final float Yp = 0f;
	public static final float Lp = -1.516f; // 1/s
	public static final float Lr = 0.874f; // 1/s
	public static final float Nbeta = 18.78f; // 1/s2
	public static final float Np = -0.040f; // 1/s
	public static final float Yr = 0f;
	public static final float Nr = -0.566f; // 1/s
	public static final float Ydr = 25.09f; // ft/s^2
	public static final float Ldr = 9.961f; // 1/s2
	public static final float Ndr = -8.397f; // 1/s2
	public static final float Yda = -2.409f; // ft/s2
	public static final float Lda = 21.27f; // 1/s2
	public static final float Nda = 0.479f; //1/s2
	public static final float Lprimebeta = -34.90f; // 1/s2
	public static final float Lprimep = -1.516f; // 1/s
	public static final float Lprimer = 0.872f; // 1/s
	public static final float Nprimebeta = 18.73f; // 1/s2
	public static final float Nprimep = 0.038f; // 1/s
	public static final float Nprimer = -0.565f; // 1/s
	public static final float Lprimedr = 9.918f; // 1/s2
	public static final float Nprimedr = -8.383f; // 1/s2
	public static final float Lprimeda = 21.27f; // 1/s2
	public static final float Nprimeda = 0.508f; // 1/s2

	//// Table B.1d Eigenvalue Summary
	// Longitudinal
	public static final float xisp = 0.301f; 
	public static final float omegasp = 3.702f; // 1/s
	public static final float xip = 0.087f; // 1/s
	public static final float omegap = 0.076f; // 1/s
	// Lateral
	public static final float xid = 0.089f;
	public static final float omagad = 4.340f; // 1/s
	public static final float oneoTr = 1.535f; // 1/s
	public static final float oneoTs = 0.006f; // 1/s
}
