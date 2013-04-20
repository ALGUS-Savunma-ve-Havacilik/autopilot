package com.example.autopilot;

public class Matrix {
	// return a random m-by-n matrix with values between 0 and 1
    public static float[][] random(int m, int n) {
        float[][] C = new float[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[i][j] = (float) Math.random();
        return C;
    }

    // return n-by-n identity matrix I
    public static float[][] identity(int n) {
        float[][] I = new float[n][n];
        for (int i = 0; i < n; i++)
            I[i][i] = 1;
        return I;
    }

    // return x^T y
    public static float dot(float[] x, float[] y) {
        if (x.length != y.length) throw new RuntimeException("Illegal vector dimensions.");
        float sum = 0.0f;
        for (int i = 0; i < x.length; i++)
            sum += x[i] * y[i];
        return sum;
    }

    // return C = A^T
    public static float[][] transpose(float[][] A) {
        int m = A.length;
        int n = A[0].length;
        float[][] C = new float[n][m];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[j][i] = A[i][j];
        return C;
    }

    // return C = A + B
    public static float[][] add(float[][] A, float[][] B) {
        int m = A.length;
        int n = A[0].length;
        float[][] C = new float[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[i][j] = A[i][j] + B[i][j];
        return C;
    }

    // return C = A - B
    public static float[][] subtract(float[][] A, float[][] B) {
        int m = A.length;
        int n = A[0].length;
        float[][] C = new float[m][n];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                C[i][j] = A[i][j] - B[i][j];
        return C;
    }

    // return C = A * B
    public static float[][] multiply(float[][] A, float[][] B) {
        int mA = A.length;
        int nA = A[0].length;
        int mB = B.length;
        int nB = A[0].length;
        if (nA != mB) throw new RuntimeException("Illegal matrix dimensions.");
        float[][] C = new float[mA][nB];
        for (int i = 0; i < mA; i++)
            for (int j = 0; j < nB; j++)
                for (int k = 0; k < nA; k++)
                    C[i][j] += (A[i][k] * B[k][j]);
        return C;
    }

    // matrix-vector multiplication (y = A * x)
    public static float[] multiply(float[][] A, float[] x) {
        int m = A.length;
        int n = A[0].length;
        if (x.length != n) throw new RuntimeException("Illegal matrix dimensions.");
        float[] y = new float[m];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                y[i] += (A[i][j] * x[j]);
        return y;
    }


    // vector-matrix multiplication (y = x^T A)
    public static float[] multiply(float[] x, float[][] A) {
        int m = A.length;
        int n = A[0].length;
        if (x.length != m) throw new RuntimeException("Illegal matrix dimensions.");
        float[] y = new float[n];
        for (int j = 0; j < n; j++)
            for (int i = 0; i < m; i++)
                y[j] += (A[i][j] * x[i]);
        return y;
    }
    
    public static float norm(float[] vector) {
    	float sum = 0;
    	for (int i = 0; i < vector.length; i = i+1){
    		sum = (float) (sum + Math.pow(vector[i],2));
    	}		
    	return (float) Math.sqrt(sum);
    }
    
    public static float [][] vectorToArray(float [] vector) {
    	int n = (int) Math.sqrt(vector.length);
    	int count = 0;
    	
    	float[][] array = new float [n][n];
    	
    	for(int i = 0;i<n;i=i+1){
    		for (int j = 0; j<n; j = j+1){
    			array[i][j] = vector[count];
    			count = count+1;
    		}
    	}
    	return array;
    }
    
    public static float [] matrixToVector(float [][] matrix) {
    	final int n = (int) matrix.length;
    	int count = 0;
    	
    	float[] vector = new float [(int) Math.pow(n, 2)];
    	
    	for (int i = 0;i<n;i=i+1){
    		for (int j = 0; j<n; j = j+1){
    			vector[count] = matrix[i][j];
    			count = count+1;
    		}
    	}
    	return vector;
    }
    
}
