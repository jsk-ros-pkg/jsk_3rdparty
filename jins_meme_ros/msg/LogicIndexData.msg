# https://jins-meme.github.io/sdkdoc2/basics/definition.html
string date # 計測日時 2000-01-01T00:00:00 - 2099-12-31T23:59:59
int32 stepCount # 歩数 0-255
int32 stepCadance # ケイデンス(ピッチ)	0-255
bool isStill # 静止（装着してない）判定	true: 非装着（静止） false: 装着中（非静止）
float32 noiseTime # ノイズ時間	0.00 - 15.00
bool isValid # 静止状態のデータ有効性（ノイズ3秒以下かつ歩数5歩以下）	true: 有効 false: 無効
float32 xMean # 傾き平均X (度)	-180.00-180.00
float32 xSD # 傾き標準偏差X (度)	0-655.36
float32 yMean # 傾き平均Y (度)	-180.00-180.00
float32 ySD # 傾き標準偏差Y (度)	0-655.36
int32 pitchOnewayCount # Number(int)	頭部運動縦回数	0-255
int32 pitchRoundCount # ゆっくりな首の傾斜前後回数	0-255
int32 yawOnewayCount # 頭部運動横回数	0-255
int32 yawRoundCount # ゆっくりな首の傾斜左右回数	0-255
float32 xRightStepAmplitude # 歩行振動X（cm,右足）	0.00-16.00
float32 xLeftStepAmplitude # 歩行振動X（cm,左足）	0.00-16.00
float32 yRightStepAmplitude # 歩行振動Y（cm,右足）	0.00-16.00
float32 yLeftStepAmplitude # 歩行振動Y（cm,左足）	0.00-16.00
float32 zRightStepAmplitude # 歩行振動Z（cm,右足）	0.00-16.00
float32 zLeftStepAmplitude # 歩行振動Z（cm,左足）	0.00-16.00
float32 zRightStepAmplitudeCal # 歩行振動Z補正有（cm,右足）	0.00-20.00
float32 zLeftStepAmplitudeCal # 歩行振動Z補正有（cm,左足）	0.00-20.00
float32 maxRightStepAcceleration # Number(float)	最大着地強度平均 (G, 右足)	0.00-8.00
float32 maxLeftStepAcceleration # 最大着地強度平均 (G, 左足)	0.00-8.00
float32 sleepScoreStandard # 低覚醒スコア(通常時)、まばたき強さ・速度と一部まばたき間隔から「目がトロンとなっている状態」を指標化したもの	有効時: 0(覚醒高い)-100(眠い)、無効: -1
float32 sleepScore # 低覚醒スコア(運転時)、運転時の姿勢(まっすぐ前を向いている状態)でクレンジングし、精度を高めたもの	有効時: 0(覚醒高い)-100(眠い)、無効: -1
float32 focusScore # 没入スコア、まばたき間隔を利用し「ある一つのタスクへの注意が続いている状態」を指標化したもの	有効時: 0(没入度が低い)-100(没入度が高い)、無効: -1
float32 tensionScore # テンションスコア、まばたき強さを利用し「目が見開いている状態」を指標化したもの	有効時: 0(緊張が弱い)-100(緊張が強い)、無効: -1
float32 calmScore # 安定スコア、まばたき強さを利用し「外部・内部刺激を受けずに安定している状態」を指標化したもの	有効時: 0(落ち着いていない)-100(落ち着いている)、無効: -1
float32 distance # サブアプリ動作時 前回区間との移動距離(m)	0-5000
float32 latitude # サブアプリ動作時 緯度	-180 - 180
float32 longitude # サブアプリ動作時 経度	-90 - 90
int32 appMeasurementStatus # サブアプリの動作状況フラグ	0: 非APP測定 2: Run測定中 3: Run一時停止中 8: Drive測定中 12: Drive一時停止中 32: Focus測定中 48: Focus一時停止中
float32 nptMean # NPT（実効まばたき速度）平均	-0.999 - 0.999
float32 nptMedian # NPT（実効まばたき速度）中央値	-0.999 - 0.999
float32 nptSD # NPT標準偏差	0-0.999
float32 blinkWidthMean # まばたき速度平均(mSec)	0-300
float32 blinkStrengthTotal # まばたき強度合計(uV-equiv)	0-10000.0
float32 blinkStrengthMax # まばたき強度最大(uV-equiv)	0-1000.0
float32 blinkStrengthSD # まばたき強度標準偏差(uV-equiv)	0.00-1000.0
float32 blinkStrengthMean # まばたき強度平均	0-1000.0
float32 blinkIntervalTotal # まばたき間隔合計(s)	0-120.0
int32 blinkIntervalCount # まばたき間隔数	0-120
float32 blinkIntervalMean # まばたき間隔平均	0.00-60.00
int32 blinkCount # まばたき回数	0-120
int32 blinkCountRaw # まばたき回数生値	0-255
int32 eyeMoveUpCount # 視線移動上回数生値	0-255
int32 eyeMoveDownCount # 視線移動下回数生値	0-255
int32 eyeMoveRightCount # 視線移動右回数生値	0-255
int32 eyeMoveLeftCount # 視線移動左回数生値	0-255
uint32 cummulativeTime # 規格化累積時間(s)	0-4294967296
float32 blinkIntervalMeanWA # まばたき間隔平均 規格化値	0.00-60.00
float32 blinkStrengtnSDWA # まばたき強度標準偏差 規格化値	0.00-1000.0
float32 blinkStrengthMeanWA # まばたき強度平均 規格化値	0-1000.0
float32 nptMeanWA # NPT（実効まばたき速度）平均 規格化値	-0.999 - 0.999
float32 nptSDWA # NPT（実効まばたき速度）標準偏差 規格化値	0-0.999
float32 blinkWidthMeanWA # まばたき速度平均 規格化値	0-300
float32 nptScore # 覚醒サブ指標 NPTスコア	有効時: 0-100、無効: -1
float32 btsScore # 覚醒サブ指標 BTSスコア	有効時: 0-100、無効: -1
float32 lbsScore # 覚醒サブ指標 LBSスコア	有効時: 0-100、無効: -1
int32 legacyZone # RTアルゴリズム動作時 zone値	有効時: 0-100、無効: -1
int32 legacyFocus # RTアルゴリズム動作時 focus値	有効時: 0-100、無効: -1
int32 legacyCalm # RTアルゴリズム動作時 calm値	有効時: 0-100、無効: -1
int32 legacyPosture # RTアルゴリズム動作時 posture値	有効時: 0-100、無効: -1
string cursor # 次のレコードがある場合の取得開始位置	 
