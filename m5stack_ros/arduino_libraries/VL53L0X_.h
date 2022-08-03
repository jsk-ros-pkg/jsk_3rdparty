// Please see
// https://note.suzakugiken.jp/pololu-tof-tutorial-a/

/*
　この例では、VL53L0Xからシングルショット範囲の測定値を取得する方法を示します。
　VL53L0X APIユーザーマニュアルに記載されているように、センサーはオプションでさまざまな測距プロファイルで設定することができ、特定のアプリケーションのパフォーマンスを向上させることができます。
　このコードは、VL53L0X APIの4つの「SingleRanging」の例に基づいています。
  測定範囲はmm単位です。
*/

/*
長距離モードを使用するには、この行のコメントを外してください。 これはセンサの感度を高め、その潜在的な範囲を広げるが、意図されたターゲット以外の物体からの反射のために不正確な読みを得る可能性を高める。 暗い場所では最も効果的です。
*/
//#define LONG_RANGE
/*
これら2行のうちの1つのコメントを外して、精度を下げてより高い速度を得るか、または速度を落としてより高い精度を得る
*/
//#define HIGH_SPEED
//#define HIGH_ACCURACY

#include <M5Stack.h>

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
uint16_t tof;

void setupVL53L0X()
{
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
#if defined LONG_RANGE
  // 戻り信号のレート制限を下げます（デフォルトは0.25 MCPS）
  sensor.setSignalRateLimit(0.1);
  // レーザーパルス周期を長くする（デフォルトは14および10 PCLKs）
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  // タイミングバジェットを20ミリ秒に減らす（デフォルトは約33ミリ秒）
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // タイミングバジェットを200ミリ秒に増やす
  sensor.setMeasurementTimingBudget(200000);
#endif

}

void measureVL53L0X()
{
  tof = sensor.readRangeSingleMillimeters();
}

void displayVL53L0X()
{
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(10, 10);

  if (sensor.timeoutOccurred())
  {
    M5.Lcd.printf("TIMEOUT  ");
  }
  else
  {
    M5.Lcd.printf("tof: %4u", tof);
  }
}
