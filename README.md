# MEGABUS Web

MEGABUS akilli metrobus projesinin Vercel uzerinden yayinlanacak statik web panelidir.

## Sayfalar

- `index.html`: Kok adresi `guest.html` yolcu paneline yonlendirir.
- `guest.html`: Yolcu paneli. Firebase'den canli arac, istasyon ve ETA verilerini okur.
- `admin.html`: Admin paneli. Firebase Auth ile giris yapar; sistem kontrolu, route calibration, manuel kontrol, PID ve car-based AI ETA egitimi/tahmini icerir.
- `firebase_structure_example.json`: Firebase Realtime Database icin ornek veri semasi.

## Vercel

Bu klasor direkt statik site olarak deploy edilebilir. Build komutu gerekmez.

Vercel ayarlari:

- Framework Preset: `Other`
- Build Command: bos birakilabilir
- Output Directory: bos birakilabilir

## Not

Arduino/ESP8266 dosyalari bu public web repo'ya dahil edilmemelidir. Arac kodunda Wi-Fi bilgileri ve cihaz ayarlari bulunabilir.
