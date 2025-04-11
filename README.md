追尾アルゴシミュレーション環境  
カルマンフィルターを用いた移動座標の追尾テスト  
  
CMakeLists.txt    Makeファイル  
KalmanFilter.cpp　カルマンフィルター本体  
KalmanFilter.h　　カルマンフィルターヘッダー  
KalmanFilterTest.cpp  main処理  
KalmanFilterTest.h    mainヘッダー

  
# 利用オープンソース  
* Eigen  
Eigenはヘッダーのみで提供されるテンプレートライブラリです。  
線形代数の演算を高速で実行できます。  
https://eigen.tuxfamily.org/index.php?title=Main_Page  
  
* Eigenインストール方法  
  
Windows  
　自分で直接管理する場合  
１．eigen-3.4.0.zipをダウンロード解凍する。  
２．\eigen-3.4.0\eigen-3.4.0\Eigenを、自身のPCのローカル上に置く。  
３．VisualStudioのインクルードパスに、上記フォルダーを追加する。  
  
　vcpkgを使う場合（VisualStudioの環境設定をしてくれるのでお勧め）  
１．vcpkgをインストールする。  
２．コマンドプロンプトを管理者として実行、以下のコマンドを実行する。  
c:\vcpkg\vcpkg search eigen  
c:\vcpkg\vcpkg install eigen3:x64-windows  
３．システムの環境変数Pahtに、次を追加する。  
　c:\vcpkg\installed\x64-windows\bin  
  
Linux　Debian系  
$ sudo apt install -y libeigen3-dev  
  
Linux　RedHat系  
$ dnf -y install eigen3-devel  
  
* OpenCV  
　各自、OpenCVからダウンロードして、ローカルPCに配置する。  
https://opencv.org/  
  
注）OpenCVのデバックモードライブラリは、メモリーリークを起こしている。  
OpenCV自体のデバックはしないので、リリースモードライブラリを、デバック/リリース双方で使用することを奨励する。  
  
OpenCVは、バージョンごとの差異を評価中なので、本プロジェクトに登録されているディレクトリは入れ子になっている。  


