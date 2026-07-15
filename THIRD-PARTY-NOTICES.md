# Third-party notices

**Airlive for OBS** is licensed under the **GNU General Public License v2.0** (see
[`LICENSE`](LICENSE)) — the standard license for an OBS Studio plugin, which links
libobs. It uses the following components, each under its own license:

| Component | Role | License |
|---|---|---|
| **libobs** (OBS Studio) | plugin host API this links against | GPL-2.0 |
| **FFmpeg** (libavcodec / libavutil / libswscale) | H.264 decode / scaling | LGPL-2.1+ / GPL |
| **Bonjour / dns_sd** | service discovery (`_airlive._tcp`) | system (Apple) / Apache-2.0 SDK on Windows |

The Airlive wire-protocol implementation bundled here (packet framing, control
messages, HMAC auth) mirrors the **[Airlive Protocol](https://github.com/airlive-project/airlive-protocol)**
(Apache-2.0), re-used here under a GPL-2.0-compatible grant.

## Trademarks

- **OBS**, **OBS Studio** are trademarks of their respective owners. This plugin
  is not affiliated with, sponsored by, or endorsed by the OBS Project.
- All other product names, logos, and brands are property of their respective
  owners and are used for identification only.
