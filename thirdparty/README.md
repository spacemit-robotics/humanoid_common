# Vendored dependencies

- `qrcodegen/`: Nayuki QR Code generator, tag `v1.8.0`, C++ sources from
  https://github.com/nayuki/QR-Code-generator/tree/v1.8.0/cpp, with trailing
  whitespace removed. Logic and the MIT license are unchanged.
- `../clients/web/vendor/lucide.min.js`: Lucide `0.468.0`, UMD bundle from the
  published package; ISC license in `LICENSE.lucide`.

HTTP/WebSocket uses the system Boost.Beast headers; JSON uses the system
nlohmann-json headers. Neither is reimplemented or copied here.
Android launcher resources use the official SpacemiT logo. See
`clients/web/assets/README.md` for the brand asset source.
