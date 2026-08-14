# MediaPipe Tasks hand tracking migration

## Runtime selection

- Default: `?hand_engine=tasks`
- Force old engine: `?hand_engine=legacy`
- Tasks tries GPU first, then CPU. If Tasks still cannot initialize, it loads Legacy.
- Both engines run entirely in the browser and send the existing `mediapipe` 21-point payload to `/ws/hand/mimic` or `/api/hand/mimic`.

The status overlay shows the actual engine/delegate, FPS, and inference P95. The
same values are available from `HandCameraControl.getMetrics()`.

## Landmark compatibility contract

The adapter preserves the values returned by MediaPipe; it does not rotate,
mirror, rescale, or relabel points before sending them to Python.

| Property | Tasks contract | Migration decision |
| --- | --- | --- |
| Point order | 21 canonical hand points: wrist `0`; thumb `1-4`; index `5-8`; middle `9-12`; ring `13-16`; little finger `17-20` | Matches the legacy topology expected by the backend |
| Image axes | normalized `x` grows left-to-right in the input image; `y` grows top-to-bottom; `z` is wrist-relative depth | Used only for the browser overlay |
| World coordinates | `x/y/z` are model-estimated 3D coordinates in meters, with origin near the hand geometric center | Forward unchanged as the retargeting input |
| Handedness | MediaPipe labels assume a mirrored/selfie input | The current video and inference input are not mirrored, so the raw label can be opposite the physical hand; the backend currently ignores this label |
| Mirroring | no CSS or canvas mirroring and no coordinate sign flip | Legacy and Tasks receive the same unmirrored pixels |

The topology, units, and documented conventions are compatible. Numerical
equivalence is **not** assumed: Tasks uses a different graph/model packaging,
so world coordinates and downstream joint angles still need paired measurement
on the same recorded 480x360 frames.

## Browser acceptance run

1. Serve the app over HTTPS and run the same 480x360 recorded motion once with
   `?hand_engine=legacy`, then once with `?hand_engine=tasks`.
2. Record the overlay metrics after warm-up. Tasks must sustain at least 27 FPS
   and inference P95 must be at most 45 ms.
3. Compare matched backend `joint_angles`; mean absolute joint error should be
   below 0.1 rad.
4. Move each physical hand out of frame and back in. Confirm reacquisition and
   check the raw handedness caveat above.
5. Disconnect WebSocket and confirm HTTP continues. Restore it and confirm the
   WebSocket reconnects without more than one request in flight.
6. Stop and rapidly restart the camera. Confirm tracks, frame callbacks, model,
   HTTP request, and WebSocket are released and no stopped connection reconnects.
7. Repeat Chrome/Edge, camera denial, and GPU-disabled runs. The overlay must
   show `tasks/CPU` or `legacy/WASM` when fallback occurs.

Automated adapter and backpressure tests:

```bash
node web/tests/hand_tracker_tasks.test.mjs
node web/tests/hand_mimic_transport.test.mjs
```
