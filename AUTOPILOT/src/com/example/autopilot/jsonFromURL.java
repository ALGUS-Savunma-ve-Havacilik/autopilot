package com.example.autopilot;

import java.io.IOException;

import org.apache.http.client.ClientProtocolException;
import org.apache.http.client.HttpClient;
import org.apache.http.client.ResponseHandler;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.BasicResponseHandler;
import org.apache.http.impl.client.DefaultHttpClient;

import android.os.AsyncTask;

class jsonFromURL extends AsyncTask<String, Void, String> {
	protected String doInBackground(String... urls) {
		HttpClient httpclient = new DefaultHttpClient();
		HttpGet httpget = new HttpGet(urls[0]);
		String responseBody = null;

		ResponseHandler<String> responseHandler = new BasicResponseHandler();
		try {
			responseBody = httpclient.execute(httpget, responseHandler);
		} catch (ClientProtocolException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		httpclient.getConnectionManager().shutdown();
		return responseBody;
	}

}