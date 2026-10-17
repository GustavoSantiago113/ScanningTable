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
import androidx.camera.core.Camera
import androidx.camera.camera2.interop.CaptureRequestOptions
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import java.io.File

class MainActivity : FlutterActivity() {
    private val CHANNEL = "com.gustavo.saf"
    private val PICK_DIRECTORY_REQUEST = 42
    private var pendingResult: MethodChannel.Result? = null
    private var analysisUseCase: ImageAnalysis? = null
    private var boundCamera: Camera? = null

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
                "copyFileToDirectory" -> {
                    val treeUri = call.argument<String>("treeUri")
                    val filename = call.argument<String>("filename")
                    val sourcePath = call.argument<String>("sourcePath")

                    if (treeUri != null && filename != null && sourcePath != null) {
                        // Videos can be hundreds of MB: stream the copy off the main thread.
                        Thread {
                            val success = copyFileToDirectory(treeUri, filename, sourcePath)
                            runOnUiThread { result.success(success) }
                        }.start()
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
                "resetCamera2Settings" -> {
                    resetCamera2Settings(result)
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

    private fun copyFileToDirectory(treeUriString: String, filename: String, sourcePath: String): Boolean {
        try {
            val treeUri = Uri.parse(treeUriString)
            val documentFile = DocumentFile.fromTreeUri(this, treeUri) ?: return false

            var file = documentFile.findFile(filename)
            if (file == null) {
                file = documentFile.createFile("application/octet-stream", filename)
            }

            if (file == null) return false

            File(sourcePath).inputStream().use { input ->
                contentResolver.openOutputStream(file.uri)?.use { output ->
                    input.copyTo(output)
                    output.flush()
                } ?: return false
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
                // Reuse the same use case so repeated calls don't stack up analyzers.
                val imageAnalysis = analysisUseCase ?: ImageAnalysis.Builder().build().also {
                    it.setAnalyzer(ContextCompat.getMainExecutor(this)) { img -> img.close() }
                    analysisUseCase = it
                }

                val camera = cameraProvider.bindToLifecycle(
                    this, CameraSelector.DEFAULT_BACK_CAMERA, imageAnalysis
                )
                boundCamera = camera

                Camera2CameraControl.from(camera.cameraControl).setCaptureRequestOptions(
                    CaptureRequestOptions.Builder()
                        // 1/4 second shutter speed = 250,000,000 nanoseconds
                        .setCaptureRequestOption(CaptureRequest.SENSOR_EXPOSURE_TIME, 250_000_000L)
                        // ISO 200
                        .setCaptureRequestOption(CaptureRequest.SENSOR_SENSITIVITY, 200)
                        // f/2.4 aperture (no-op on fixed-aperture lenses)
                        .setCaptureRequestOption(CaptureRequest.LENS_APERTURE, 2.4f)
                        .build()
                )

                result.success(true)
            } catch (e: Exception) {
                result.error("CAMERA2_ERROR", e.message, null)
            }
        }, ContextCompat.getMainExecutor(this))
    }

    // Undo applyCamera2Settings for video: drop the 1/4 s manual exposure and unbind the extra
    // ImageAnalysis use case so the camera has a free stream for the plugin's VideoCapture.
    @SuppressLint("UnsafeOptInUsageError")
    @OptIn(ExperimentalCamera2Interop::class)
    private fun resetCamera2Settings(result: MethodChannel.Result) {
        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener({
            try {
                boundCamera?.let {
                    Camera2CameraControl.from(it.cameraControl).clearCaptureRequestOptions()
                }
                analysisUseCase?.let { cameraProviderFuture.get().unbind(it) }
                boundCamera = null
                result.success(true)
            } catch (e: Exception) {
                result.error("CAMERA2_ERROR", e.message, null)
            }
        }, ContextCompat.getMainExecutor(this))
    }
}
