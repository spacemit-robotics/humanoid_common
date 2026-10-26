# Third-party dependencies

- cnpy: commit `4e8810b1a8637695171ed346ce68f6984e585ef4`, fetched and patched
  by `cmake/FindCnpy.cmake` in the build directory.
- [Nayuki QR Code generator](https://github.com/nayuki/QR-Code-generator/tree/v1.8.0/cpp):
  `v1.8.0`, commit `720f62bddb7226106071d4728c292cb1df519ceb`, fetched by
  `cmake/FindQrcodegen.cmake` and compiled with the SDK toolchain.
  The MIT notice is retained in `LICENSE.qrcodegen` and installed with the SDK.
- [Lucide](https://www.npmjs.com/package/lucide/v/0.468.0): `0.468.0`, fetched
  by `cmake/FindLucide.cmake` from the published package. The browser bundle and
  ISC license are installed under `share/humanoid_common/operator_web/vendor/`.
  The ISC notice is also retained in `LICENSE.lucide`.

Sources and packages are cached in `~/.cache/thirdparty/`. Set the environment
variable `SROBOTIS_THIRDPARTY_CACHE` to use another cache root. For offline builds,
pre-populate the cache and pass `-DSROBOTIS_THIRDPARTY_FETCH_OFF=ON` to CMake.
An incomplete cache produces a configuration error when fetching is disabled.

The complete web root is assembled under `<build>/operator_web/`; source files
remain in `clients/web/`. The installed service uses the installed web root,
with the build copy available for local development. Browser clients load the
icon bundle from the service, without contacting an external CDN at runtime.

HTTP/WebSocket uses the system Boost.Beast headers; JSON uses the system
nlohmann-json headers. Neither is reimplemented or copied here.
Android launcher resources use the official SpacemiT logo. See
`clients/web/assets/README.md` for the brand asset source.
