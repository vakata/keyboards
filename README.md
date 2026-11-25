# vakata's keyboards

<img width="1600" height="1200" alt="keyboards" src="https://github.com/user-attachments/assets/d97ad558-30f1-42b9-88ca-a78046af9a71" />

I use mainly three keyboards - the KLOR, the SWOOP and ZSA Voyager. My Swoops are from Leo on beekeeb, and the KLOR I soldered myself by using a kit from the same site. I use Vial to configure the KLOR and SWOOP.

## SWOOP

It is pretty much ready as it is from the shop, but I wanted more features that I had to fit inside the memory constraints of the MCU. My custom firware and config is here.

```
git clone https://github.com/vial-kb/vial-qmk.git
cd vial-qmk
git checkout --detach dd43959ae5c08d8a28d38a1acf7b04e86b14a344
git submodule update --init --recursive
ln -s <THIS-REPO>/firmware/vakata ./keyboards/vakata
qmk compile -kb vakata/swoop -km vial
```

## KLOR 

Since I already own the Navigator trackpad I decided to try and port it to the KLOR by attaching it to the haptic motor ports (so the same i2C bus) on the right half. The tip of the TRRS connector is VCC, the sleeve is GND, ring 1 is SCL, and ring 2 is SDA. With some help from AI it works for my basic needs (pointing, clicking, two finger scrolling).

There is also a base for the KLOR in the 3DP folder. It was created with AI. The base needs small M2 screws.

```
git clone https://github.com/vial-kb/vial-qmk.git
cd vial-qmk
git checkout --detach dd43959ae5c08d8a28d38a1acf7b04e86b14a344
git submodule update --init --recursive
ln -s <THIS-REPO>/firmware/vakata ./keyboards/vakata
qmk flash -kb vakata/klor -km vial -bl uf2-split-left
qmk flash -kb vakata/klor -km vial -bl uf2-split-right
```

## KEYCAPS

I use the KLP lame keycaps - there is a 3mf file in the 3DP folder. I used a Bambu H2S with 0.2mm nozzle for the print. 
