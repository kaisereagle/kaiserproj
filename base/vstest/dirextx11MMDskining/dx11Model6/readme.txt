Zerogramサンプルプログラム     2013/4/14

■スキニングと頂点ブレンド■
・頂点シェーダでスキニングと頂点ブレンド
・可変サイズの定数バッファ対応
・PMD/VMDの表示再生（一部の機能のみ）

■ファイル
sample_data.h     ：使用するデータのファイルパス設定
zg_dx11.h/cpp     ：DX11関連＆PMD/VMD表示再生
zg_mmd.h/cpp      ：PMD/VMD関連
shader/           ：シェーダソースコード
data/             ：データ　PMD/VMDファイルは含まれません

■使用ライブラリ
サンプルプログラムには含まれていません。
個別に取得して、指定された場所に配置してください。

○DirectXTex
http://go.microsoft.com/fwlink/?LinkId=248926
DirectX11で画像ファイルを扱うためのライブラリ
ソースコードで公開、自分でコンパイルして使用
ヘッダ、ライブラリファイルの配置場所は、下記参照

○Boost C++ Library
http://www.boost.org/
ver1.53

■ライブラリファイル構成
+ dx11Model6
   + dx11Model6
     VSプロジェクト、ソースコードなど
     + shader
        シェーダソースコード
     + data
        データファイル
+ library
   + include
      + boost  //boost
      + DirectXTex //DirextXTexライブラリからコピー
         - DirectXTex.h
         - DirectXTex.inl
   + lib               //リリースビルド用libファイル
      - DirectXTex.lib // コンパイルして作成（Release)
   + lib_debug         //デバッグビルド用libファイル
      - DirectXTex.lib // コンパイルして作成（Debug）

ライブラリファイルの参照は、VSプロジェクトファイルからの相対パスで指定しています。
別の場所に配置する場合は、インクルードとリンクの追加ディレクトリを変更してください。

■ツールなど
○MikuMikuDance
VPVP
http://www.geocities.jp/higuchuu4/index.htm

○PMD/VMDフォーマット
MMDのモデルデータ(PMD)形式　めも　(まとめ)/通りすがりの記憶
http://blog.goo.ne.jp/torisu_tetosuki/e/209ad341d3ece2b1b4df24abf619d6e4

○VMD用のベジェ曲線計算
カメラとライトと表情のモーションに対応（あと補間曲線について）
http://d.hatena.ne.jp/edvakf/20111016/1318716097

----------------------------------------------
ゼログラム
http://zerogram.info

■このサンプルプログラムによって生じた損失や損害などについては、いかなる場合も一切の責任を負いません。
サンプルプログラムの使用に際しては、個人の責任で行ってください。
