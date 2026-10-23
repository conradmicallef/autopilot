# Cheap foam airframes for testing the autopilot

Prices are approximate, as of October 2026. No Amazon listings could be verified, so the Amazon links below are searches.

## Airframes

### ZOHD Drift, 877 mm EPP: best cheap pick
- Tough foam, glides well, flies slowly.
- No wheels: hand launch and belly landing.
- About €60 as a kit, €100–130 PNP (motor, ESC and servos fitted).
- Shops:
  - [racer.lt (EU)](https://www.racer.lt/item/zohd-drift-877mm-wingspan-fpv-glider-aio-epp-rc-airplane-kit-pnp-fpv-version-28882.html)
  - [GetFPV](https://www.getfpv.com/zohd-drift-fpv-glider-877mm-wingspan-aio-epp-rc-airplane-kit-pnp-fpv.html)
  - [Motion RC](https://MotionRc.com/collections/rc-gliders/products/zohd-drift-glider-877mm-34-52-pnp-zoh10060)
  - [Amazon.it search](https://www.amazon.it/s?k=ZOHD+Drift)

### Volantex Ranger 1600 (757-7), 1.6 m: most C172-like
- Conventional tail with a rudder, flies slowly. The crosswind and sideslip logic carries over best.
- About €130–170 PNP.
- Shops:
  - [HobbyKing Italy](https://hobbyking.com/it_it/volantex-757-7-ranger-1600mm-fpv-glider-63-pnf.html)
  - [specs](https://www.skyraccoon.com/aircraft/Volantex-RC_Ranger-1600_757-7)
  - [Amazon.it search](https://www.amazon.it/s?k=Volantex+Ranger+1600)
- Smaller alternative: the Ranger 757-4, 1380 mm ([review](https://www.flitetest.com/articles/volantex-ranger-757-4-1380mm-fpv-endurance-plane)).

## Electronics (about €150–250)

| Part | Example | Approx. price | Notes |
|---|---|---|---|
| Flight controller | Matek H743-WING (ArduPilot) ([GetFPV](https://www.getfpv.com/planes/electronics/mateksys-h743-wing-v3-flight-controller.html)) | — | The V3 is listed as discontinued at some UK shops; check for its successor. The F405-WING class also runs ArduPlane |
| GPS with compass | Matek M10Q | €25–35 | |
| Airspeed sensor | Matek ASPD-4525 | ~€40 | Optional at first |
| Lidar rangefinder | Benewake TF-Luna or TF-Mini | €25–40 | For the flare |
| Radio and battery | Receiver, transmitter, 3S LiPo | — | |

## Avoid for this purpose

- **Toy-class ready-to-fly planes under ~€100:** no room or access for a real flight controller and sensors.
- **Flying wings:** no rudder or tail, so less like the C172 logic.

## Suggestion

A ZOHD Drift PNP with a Matek wing flight controller, flown with ArduPlane. Test in SITL first, then follow the flight-test ladder in `foam-rc-autopilot.md`.

Further reading:
- [RC Plane DIY on GPS and autopilot systems](https://rcplanediy.com/2026/03/10/gps-autopilot-rc-airplanes/)
- [Aeronautics Magazine foamie roundup](https://aeronauticsmagazine.com/hobby/rc-planes/best-foamie-rc-planes)
