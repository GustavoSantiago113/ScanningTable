package com.example.scanning_table_app

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
import android.annotation.SuppressLint
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import androidx.camera.camera2.interop.Camera2CameraControl
import androidx.camera.camera2.interop.CaptureRequestOptions
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat

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
                "applyCamera2Settings" -> {
                    applyCamera2Settings(result)
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

    @SuppressLint("UnsafeOptInUsageError")
    @OptIn(ExperimentalCamera2Interop::class)
    private fun applyCamera2Settings(result: MethodChannel.Result) {
        // Check that the back camera supports manual sensor control.
        val cameraManager = getSystemService(CAMERA_SERVICE) as CameraManager
        val backCameraId = cameraManager.cameraIdList.firstOrNull { id ->
            val chars = cameraManager.getCameraCharacteristics(id)
            chars.get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK
        }
        if (backCameraId != null) {
            val capabilities = cameraManager
                .getCameraCharacteristics(backCameraId)
                .get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES) ?: intArrayOf()
            if (!capabilities.contains(
                    CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR
                )
            ) {
                result.error("NO_MANUAL_SENSOR", "Camera does not support manual sensor control", null)
                return
            }
        }

        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener({
            try {
                val cameraProvider = cameraProviderFuture.get()

                // Bind a no-op ImageAnalysis to obtain the Camera object.
                // CameraX 1.1+ bindToLifecycle is additive — Flutter's Preview &
                // ImageCapture use cases remain bound alongside this one.
                val imageAnalysis = ImageAnalysis.Builder().build()
                imageAnalysis.setAnalyzer(ContextCompat.getMainExecutor(this)) { it.close() }

                val camera = cameraProvider.bindToLifecycle(
                    this, CameraSelector.DEFAULT_BACK_CAMERA, imageAnalysis
                )

                Camera2CameraControl.from(camera.cameraControl)
                    .captureRequestOptions = CaptureRequestOptions.Builder()
                        // 1/4 second shutter speed = 250,000,000 nanoseconds
                        .setCaptureRequestOption(CaptureRequest.SENSOR_EXPOSURE_TIME, 250_000_000L)
                        // ISO 200
                        .setCaptureRequestOption(CaptureRequest.SENSOR_SENSITIVITY, 200)
                        // f/2.4 aperture (no-op on fixed-aperture lenses)
                        .setCaptureRequestOption(CaptureRequest.LENS_APERTURE, 2.4f)
                        .build()

                result.success(true)
            } catch (e: Exception) {
                result.error("CAMERA2_ERROR", e.message, null)
            }
        }, ContextCompat.getMainExecutor(this))
    }
}
