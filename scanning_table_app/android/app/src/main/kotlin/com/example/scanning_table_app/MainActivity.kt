package com.example.scanning_table_app

import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.os.Build
import android.os.Bundle
import java.io.File
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel
import androidx.documentfile.provider.DocumentFile

class MainActivity : FlutterActivity() {
	private val CHANNEL = "com.gustavo.saf"
	private val REQUEST_CODE_PICK_DIR = 1001
	private var pendingResult: MethodChannel.Result? = null

	override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
		super.configureFlutterEngine(flutterEngine)
		MethodChannel(flutterEngine.dartExecutor.binaryMessenger, CHANNEL).setMethodCallHandler { call, result ->
			when (call.method) {
				"pickDirectory" -> {
					pendingResult = result
					val intent = Intent(Intent.ACTION_OPEN_DOCUMENT_TREE)
					intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_WRITE_URI_PERMISSION)
					startActivityForResult(intent, REQUEST_CODE_PICK_DIR)
				}
				"saveFileToDirectory" -> {
					val treeUriStr = call.argument<String>("treeUri")
					val filename = call.argument<String>("filename")
					val dataB64 = call.argument<String>("base64")
					if (treeUriStr == null || filename == null || dataB64 == null) {
						result.error("ARGS", "Missing arguments", null)
						return@setMethodCallHandler
					}
					try {
						val bytes = android.util.Base64.decode(dataB64, android.util.Base64.DEFAULT)
						val treeUri = Uri.parse(treeUriStr)
						val docFile = DocumentFile.fromTreeUri(this, treeUri)
						val newFile = docFile?.createFile("image/jpeg", filename)
						if (newFile == null) {
							result.error("CREATE_FAIL", "Could not create file", null)
							return@setMethodCallHandler
						}
						contentResolver.openOutputStream(newFile.uri)?.use { os ->
							os.write(bytes)
							os.flush()
						}
						// Return boolean success so the Dart side can handle UI feedback
						result.success(true)
					} catch (e: Exception) {
						result.error("SAVE_ERR", e.localizedMessage, null)
					}
				}
				"openDownloads" -> {
					try {
						if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
							val downloads = File("/storage/emulated/0/Download")
							val uri = Uri.fromFile(downloads)
							val intent = Intent(Intent.ACTION_VIEW)
							intent.setDataAndType(uri, "resource/folder")
							intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
							startActivity(intent)
						} else {
							val intent = Intent(Intent.ACTION_VIEW)
							intent.setDataAndType(Uri.parse("file:///storage/emulated/0/Download"), "resource/folder")
							intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
							startActivity(intent)
						}
						result.success(true)
					} catch (e: Exception) {
						result.error("OPEN_ERR", e.localizedMessage, null)
					}
				}
				else -> result.notImplemented()
			}
		}
	}

	override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
		super.onActivityResult(requestCode, resultCode, data)
		if (requestCode == REQUEST_CODE_PICK_DIR) {
			if (resultCode == Activity.RESULT_OK && data != null) {
				val treeUri: Uri? = data.data
				if (treeUri != null) {
					contentResolver.takePersistableUriPermission(treeUri, Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_WRITE_URI_PERMISSION)
					pendingResult?.success(treeUri.toString())
				} else {
					pendingResult?.success(null)
				}
			} else {
				pendingResult?.success(null)
			}
			pendingResult = null
		}
	}
}
