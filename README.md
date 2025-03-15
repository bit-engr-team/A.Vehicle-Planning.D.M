# A.Vehicle-Planning.D.M

Bu repo, bit takımının planlama ekibinin üzerinde çalıştığı kodları içermektedir. Proje, Python 3.12.3 ile yazılmıştır ve aşağıdaki adımları takip ederek tavsiye edilen kurulumu yapabilirsiniz.

## Gereksinimler

- Python 3.12.3 veya daha yeni bir sürüm
- `pip` (Python paket yöneticisi)
- Sanal ortam (venv) kullanımı önerilir

## Kurulum

### 1. Sanal Ortam (venv) Oluşturma ve Aktifleştirme

- **Windows**:
  ```bash
  python -m venv venv
  .\venv\Scripts\activate
  ```

- **Mac/Linux**:
  ```bash
  python3 -m venv venv
  source venv/bin/activate
  ```

### 2. Gereksinimleri Yükleme

Projede kullanılan bağımlılıkları yüklemek için:

```bash
pip install -r requirements.txt
```

### 3. Bağımlılıkları Güncelleme

Yeni paketler yüklediyseniz, `requirements.txt` dosyasını güncellemek için:

```bash
pip freeze > requirements.txt
```

### Sanal Ortamı Devre Dışı Bırakma

Proje işi bittiğinde, sanal ortamı devre dışı bırakmak için:

```bash
deactivate
```

## Lisans

Bu proje [MIT Lisansı](LICENSE) altında lisanslanmıştır.
