# https://jins-meme.github.io/sdkdoc2/basics/definition.html
int32 blinkSpeed # まばたき速度、閉眼時間(mSec) 0-400(通常90−180付近)
int32 blinkStrength # まばたき強度(uV-equiv)    0-1000(通常30−150付近)
int32 eyeMoveUp # 視線移動上(0: 動きなし, 1:極小-7:極大)
int32 eyeMoveDown # 視線移動下(0: 動きなし, 1:極小-7:極大)
int32 eyeMoveLeft # 視線移動左(0: 動きなし, 1:極小-7:極大)
int32 eyeMoveRight # 視線移動右(0: 動きなし, 1:極小-7:極大)
float32 roll # 姿勢角のロール成分（左右傾き）を示す度	-180.00 - 180.00
float32 pitch # 角度P(-180 - 180)
float32 yaw # 角度Y(0 - 360)
float32 accX # 加速度X(1G = 16)
float32 accY # 加速度Y(1G = 16)
float32 accZ # 加速度Z(1G = 16)
int32 walking # 歩行フラグ(0: 静止, 1: 一歩)
int32 noiseStatus # ノイズ状態(0: ノイズなし, 1: ノイズあり)
int32 fitError # 装着フラグ(0: 着用, 1: 非着用)
int32 powerLeft # バッテリーレベル(0: 充電中, 1:空-5:満充電)
int32 sequenceNumber # 0-255までの循環連番
