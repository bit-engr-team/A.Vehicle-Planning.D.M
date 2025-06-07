# GNSS Kavşak Bağlantı Grafiği Oluşturucu

Projenin bu kısmı, GNSS (küresel konumlama sistemi) verilerini kullanarak yol üzerindeki **kavşak noktalarını** tespit eder ve bu noktalar arasında **geometrik ilişkilere** (mesafe ve açı) göre bir **bağlantı grafiği** oluşturur. Oluşturulan grafik, şehir haritalarının ağ yapısını modellemek veya rota planlama gibi uygulamalarda kullanılmak üzere görselleştirilir.

## Amaç

- GNSS verilerinden kavşakları otomatik olarak tespit etmek
- Kavşaklar arasında yönsüz bağlantılar kurmak
- Elde edilen ağı görselleştirerek analiz edilebilir hale getirmek

## Kullanılan Teknolojiler

- **Python 3**
- **pandas** – Veri işleme
- **numpy** – Sayısal hesaplamalar
- **networkx** – Grafik veri yapısı
- **matplotlib** – Görselleştirme

## Nasıl Kullanılır?

1. Jupyter Notebook ortamında `graph_new.ipynb` dosyasını açın.
2. Gerekli kütüphaneleri yükleyin (pip ile).
3. Notebook’taki hücreleri sırayla çalıştırın.
4. Sonuç olarak, kavşak noktalarının birbirine nasıl bağlandığını gösteren bir grafik elde edersiniz.

## Özellikler

- Basit ve yorumlu kod yapısı
- Kavşaklar arasında açı/mekânsal yakınlığa dayalı bağ kurulması
- Ölçeklenebilirlik: Daha büyük GNSS veri kümeleriyle çalışabilir
- Harici araçlarla kolayca entegre edilebilir

## Görselleştirme

Notebook sonunda oluşturulan grafik, kavşakları düğüm (node), yolları ise kenar (edge) olarak temsil eder. Bu yapı, şehir ağlarının modellenmesi ve analiz edilmesi için temel bir yapı sunar.



