/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
package com.spacemit.operator

import android.graphics.Color
import android.net.Uri
import android.os.Bundle
import android.os.Build
import android.text.InputType
import android.view.WindowInsets
import android.view.Gravity
import android.view.View
import android.view.inputmethod.EditorInfo
import android.webkit.WebResourceRequest
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.FrameLayout
import android.widget.ImageView
import android.widget.Toast
import android.widget.TextView
import androidx.activity.ComponentActivity
import androidx.activity.result.contract.ActivityResultContracts
import com.journeyapps.barcodescanner.ScanContract
import com.journeyapps.barcodescanner.ScanOptions
import kotlin.math.roundToInt
import java.net.URLDecoder

/** Android pairing/lifecycle shell; controls use the server's versioned web client. */
class OperatorActivity : ComponentActivity() {
    private lateinit var browser: WebView
    private lateinit var address: EditText
    private lateinit var welcome: LinearLayout
    private var pairingUrl: String? = null
    private var origin: Uri? = null
    private var pendingQr: ByteArray? = null
    private val saveQr = registerForActivityResult(ActivityResultContracts.CreateDocument("image/svg+xml")) { uri ->
        val bytes = pendingQr
        pendingQr = null
        if (uri != null && bytes != null) {
            try {
                contentResolver.openOutputStream(uri)?.use { it.write(bytes) }
                    ?: throw IllegalStateException("Cannot open selected document")
                Toast.makeText(this, "二维码已保存", Toast.LENGTH_SHORT).show()
            } catch (_: Exception) {
                Toast.makeText(this, "无法保存二维码", Toast.LENGTH_LONG).show()
            }
        }
    }
    private val scanner = registerForActivityResult(ScanContract()) { result ->
        result.contents?.let { connect(it) }
    }
    private fun dp(value: Int) = (value * resources.displayMetrics.density).roundToInt()
    private fun scan() {
        scanner.launch(ScanOptions()
            .setDesiredBarcodeFormats(ScanOptions.QR_CODE)
            .setPrompt("扫描机器人连接二维码")
            .setBeepEnabled(false)
            .setOrientationLocked(false))
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val layout = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(Color.rgb(23, 25, 24))
        }
        layout.addView(TextView(this).apply {
            text = getString(R.string.app_title)
            setTextColor(Color.rgb(214, 239, 178))
            textSize = 17f
            setPadding(dp(18), dp(16), dp(18), dp(8))
        })
        layout.setOnApplyWindowInsetsListener { view, insets ->
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                val bars = insets.getInsets(WindowInsets.Type.systemBars() or WindowInsets.Type.displayCutout() or WindowInsets.Type.ime())
                view.setPadding(bars.left, bars.top, bars.right, bars.bottom)
            } else {
                @Suppress("DEPRECATION")
                view.setPadding(insets.systemWindowInsetLeft, insets.systemWindowInsetTop,
                    insets.systemWindowInsetRight, insets.systemWindowInsetBottom)
            }
            insets
        }
        val bar = LinearLayout(this).apply { setPadding(dp(12), dp(8), dp(12), dp(8)) }
        address = EditText(this).apply {
            hint = "设备连接链接"
            setTextColor(Color.WHITE)
            setHintTextColor(Color.rgb(153, 166, 146))
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            imeOptions = EditorInfo.IME_FLAG_NO_PERSONALIZED_LEARNING
            isSingleLine = true
        }
        bar.addView(address, LinearLayout.LayoutParams(0, dp(48), 1f))
        bar.addView(Button(this).apply {
            text = "连接"
            setTextColor(Color.WHITE)
            backgroundTintList = android.content.res.ColorStateList.valueOf(Color.rgb(56, 66, 49))
            setOnClickListener { connect(address.text.toString()) }
        })
        bar.addView(Button(this).apply {
            text = "扫码"
            setTextColor(Color.rgb(30, 40, 20))
            backgroundTintList = android.content.res.ColorStateList.valueOf(Color.rgb(176, 226, 55))
            setOnClickListener { scan() }
        })
        layout.addView(bar)
        browser = WebView(this).apply {
            setBackgroundColor(Color.rgb(245, 246, 245))
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true
            settings.allowFileAccess = false
            settings.allowContentAccess = false
            settings.mixedContentMode = WebSettings.MIXED_CONTENT_NEVER_ALLOW
            settings.userAgentString += " SpaceMITOperatorApp"
            setDownloadListener { url, _, _, _, _ ->
                if (url.startsWith("data:image/svg+xml;charset=utf-8,") && url.length <= 131072) {
                    try {
                        pendingQr = URLDecoder.decode(url.substringAfter(','), "UTF-8").toByteArray(Charsets.UTF_8)
                        saveQr.launch("spacemit-operator-qr.svg")
                    } catch (_: IllegalArgumentException) {
                        Toast.makeText(this@OperatorActivity, "二维码格式无效", Toast.LENGTH_LONG).show()
                    }
                }
            }
            webViewClient = object : WebViewClient() {
                override fun shouldOverrideUrlLoading(view: WebView, request: WebResourceRequest): Boolean {
                    val target = request.url
                    val current = origin ?: return true
                    return target.scheme != current.scheme || target.host != current.host || target.port != current.port
                }
            }
        }
        browser.visibility = View.GONE
        welcome = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER
            setPadding(dp(24), dp(24), dp(24), dp(24))
            addView(ImageView(context).apply {
                setImageResource(R.drawable.spacemit_logo)
                contentDescription = "SpacemiT"
                scaleType = ImageView.ScaleType.FIT_CENTER
            }, LinearLayout.LayoutParams(dp(237), dp(72)))
            addView(TextView(context).apply {
                text = "设备未连接"
                textSize = 20f
                gravity = Gravity.CENTER
                setTextColor(Color.WHITE)
                setPadding(0, dp(32), 0, dp(24))
            })
            addView(Button(context).apply {
                text = "扫描连接二维码"
                isAllCaps = false
                setTextColor(Color.rgb(30, 40, 20))
                backgroundTintList = android.content.res.ColorStateList.valueOf(Color.rgb(176, 226, 55))
                setOnClickListener { scan() }
            }, LinearLayout.LayoutParams(-1, dp(56)))
        }
        val content = FrameLayout(this).apply {
            addView(browser, FrameLayout.LayoutParams(-1, -1))
            addView(welcome, FrameLayout.LayoutParams(-1, -1))
        }
        layout.addView(content, LinearLayout.LayoutParams(-1, 0, 1f))
        setContentView(layout)
    }

    private fun connect(raw: String) {
        val uri = Uri.parse(raw.trim())
        if (uri.scheme !in listOf("http", "https") || uri.host.isNullOrEmpty() || uri.userInfo != null) {
            Toast.makeText(this, "请输入有效的 HTTP/HTTPS 服务地址", Toast.LENGTH_LONG).show()
            return
        }
        disconnect()
        pairingUrl = uri.toString()
        origin = uri
        // The credential stays in memory, not preferences, logs or the visible address field.
        address.setText(uri.buildUpon().fragment(null).build().toString())
        welcome.visibility = View.GONE
        browser.visibility = View.VISIBLE
        browser.loadUrl(pairingUrl!!)
    }

    private fun disconnect() {
        if (::browser.isInitialized) browser.evaluateJavascript("window.operatorDisconnect?.()", null)
    }

    override fun onPause() {
        disconnect()
        browser.onPause()
        super.onPause()
    }

    override fun onResume() {
        super.onResume()
        if (::browser.isInitialized) {
            browser.onResume()
            if (pairingUrl != null) browser.evaluateJavascript("window.operatorReconnect?.()", null)
        }
    }

    override fun onDestroy() {
        disconnect()
        pairingUrl = null
        browser.destroy()
        super.onDestroy()
    }
}
