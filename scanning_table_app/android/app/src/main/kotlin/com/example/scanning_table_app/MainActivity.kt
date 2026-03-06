package com.example.gnsolutions_app

import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.os.Build
import android.provider.DocumentsContract
import android.util.Base64
import androidx.documentfile.provider.DocumentFile
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.MethodChannel

class MainActivity : FlutterActivity() {
    private val CHANNEL = "com.gustavo.saf"
    private val PICK_DIRECTORY_REQUEST = 42
    private var pendingResult: MethodChannel.Result? = null

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        
        MethodChannel(flutterEngine.dartExecutor.binaryMessenger, CHANNEL).setMethodCallHandler { call, result ->
            when (call.method) {
                "pickDirectory" -> {
                    pendingResult = result
                    pickDirectory()
                }
                "saveFileToDirectory" -> {
                    val treeUri = call.argument<String>("treeUri")
                    val filename = call.argument<String>("filename")
                    val base64Data = call.argument<String>("base64")
                    
                    if (treeUri != null && filename != null && base64Data != null) {
                        val success = saveFileToDirectory(treeUri, filename, base64Data)
                        result.success(success)
                    } else {
                        result.error("INVALID_ARGS", "Missing required arguments", null)
                    }
                }
                "openDownloads" -> {
                    openDownloadsFolder()
                    result.success(true)
                }
                else -> {
                    result.notImplemented()
                }
            }
        }
    }

    private fun pickDirectory() {
        val intent = Intent(Intent.ACTION_OPEN_DOCUMENT_TREE).apply {
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
            addFlags(Intent.FLAG_GRANT_WRITE_URI_PERMISSION)
            addFlags(Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION)
            addFlags(Intent.FLAG_GRANT_PREFIX_URI_PERMISSION)
        }
        startActivityForResult(intent, PICK_DIRECTORY_REQUEST)
    }

    private fun saveFileToDirectory(treeUriString: String, filename: String, base64Data: String): Boolean {
        try {
            val treeUri = Uri.parse(treeUriString)
            val documentFile = DocumentFile.fromTreeUri(this, treeUri) ?: return false
            
            // Create or get the file (treat as generic binary so the
            // extension provided in "filename" is preserved as-is)
            var file = documentFile.findFile(filename)
            if (file == null) {
                file = documentFile.createFile("application/octet-stream", filename)
            }
            
            if (file == null) return false
            
            // Decode base64 and write to file
            val bytes = Base64.decode(base64Data, Base64.DEFAULT)
            contentResolver.openOutputStream(file.uri)?.use { outputStream ->
                outputStream.write(bytes)
                outputStream.flush()
            }
            
            return true
        } catch (e: Exception) {
            e.printStackTrace()
            return false
        }
    }

    private fun openDownloadsFolder() {
        try {
            val intent = Intent(Intent.ACTION_VIEW).apply {
                setDataAndType(Uri.parse("content://com.android.externalstorage.documents/document/primary:Download"), 
                    DocumentsContract.Document.MIME_TYPE_DIR)
                addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
            }
            startActivity(intent)
        } catch (e: Exception) {
            // Fallback to file manager
            try {
                val intent = Intent(Intent.ACTION_GET_CONTENT).apply {
                    type = "*/*"
                }
                startActivity(Intent.createChooser(intent, "Open Downloads"))
            } catch (ex: Exception) {
                ex.printStackTrace()
            }
        }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        
        if (requestCode == PICK_DIRECTORY_REQUEST && resultCode == Activity.RESULT_OK) {
            data?.data?.let { uri ->
                // Take persistable permission
                contentResolver.takePersistableUriPermission(
                    uri,
                    Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_WRITE_URI_PERMISSION
                )
                pendingResult?.success(uri.toString())
            } ?: run {
                pendingResult?.success(null)
            }
            pendingResult = null
        } else if (requestCode == PICK_DIRECTORY_REQUEST) {
            pendingResult?.success(null)
            pendingResult = null
        }
    }
}
