# Residual Risks

- This is one display-backed NVIDIA renderer fallback in a newly owned runtime copy. It demonstrates the saved `room-1` then `hub` (`mian`) route under this environment; it does not establish robustness across hardware, driver versions, maps, or routes.
- The GLX probe passed direct NVIDIA rendering and OpenGL 4.6. `eglinfo` returned nonzero because its X11-platform EGL initialization failed, although its device platform identified NVIDIA. The actual display-backed simulator and both route goals succeeded.
- The launch log contains one transient controller progress warning before the final successful route actions. No tuning or retry was performed.
- The existing consumer's fixed zero `/initialpose` and position-only target semantics were retained. Results should not be generalized beyond that saved setup.
