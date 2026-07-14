#!/usr/bin/env node
'use strict';

const rosnodejs = require('rosnodejs');
const jins_meme_ros = rosnodejs.require('jins_meme_ros').msg;

const server = require('ws').Server;
const ws_server = new server({ port: 5000 });


function main() {
    rosnodejs.initNode('/').then((rosNode) => {
        let pub_current_data = rosNode.advertise('current_data', jins_meme_ros.CurrentData);
        let pub_logic_index_data = rosNode.advertise('logic_index_data', jins_meme_ros.LogicIndexData);
        ws_server.on("connection", ws => {
            rosnodejs.log.info("connected from client");
            ws.on('message', function (message) {
                if (message.indexOf("heartbeat") === -1) {
                    let data = JSON.parse(message);
                    if ("eyeMoveUp" in data) {
                        let msg = new jins_meme_ros.CurrentData();
                        msg.blinkSpeed = data["blinkSpeed"];
                        msg.blinkStrength = data["blinkStrength"];
                        msg.eyeMoveUp = data["eyeMoveUp"];
                        msg.eyeMoveDown = data["eyeMoveDown"];
                        msg.eyeMoveLeft = data["eyeMoveLeft"];
                        msg.eyeMoveRight = data["eyeMoveRight"];
                        msg.roll = data["roll"];
                        msg.pitch = data["pitch"];
                        msg.yaw = data["yaw"];
                        msg.accX = data["accX"];
                        msg.accY = data["accY"];
                        msg.accZ = data["accZ"];
                        msg.walking = data["walking"];
                        msg.noiseStatus = data["noiseStatus"];
                        msg.fitError = data["fitError"];
                        msg.powerLeft = data["powerLeft"];
                        msg.sequenceNumber = data["sequenceNumber"];
                        pub_current_data.publish(msg);
                        rosnodejs.log.info("published.");
                    } else if (false) {
                        let msg = new jins_meme_ros.LogicIndexData();
                        msg.date = data["date"];
                        msg.stepCount = data["stepCount"];
                        msg.blinkCountRaw = data["blinkCountRaw"];
                        msg.eyeMoveUpCount = data["eyeMoveUpCount"];
                        msg.eyeMoveDownCount = data["eyeMoveDownCount"];
                        msg.eyeMoveRightCount = data["eyeMoveRightCount"];
                        msg.eyeMoveLeftCount = data["eyeMoveLeftCount"];
                        msg.isStill = data["isStill"];
                        msg.xMean = data["xMean"];
                        msg.xSD = data["xSD"];
                        msg.yMean = data["yMean"];
                        msg.ySD = data["ySD"];
                        msg.pitchOnewayCount = data["pitchOnewayCount"];
                        msg.pitchRoundCount = data["pitchRoundCount"];
                        msg.yawOnewayCount = data["yawOnewayCount"];
                        msg.yawRoundCount = data["yawRoundCount"];
                        msg.xRightStepAmplitude = data["xRightStepAmplitude"];
                        msg.xLeftStepAmplitude = data["xLeftStepAmplitude"];
                        msg.yRightStepAmplitude = data["yRightStepAmplitude"];
                        msg.yLeftStepAmplitude = data["yLeftStepAmplitude"];
                        msg.zRightStepAmplitude = data["zRightStepAmplitude"];
                        msg.zLeftStepAmplitude = data["zLeftStepAmplitude"];
                        msg.zRightStepAmplitudeCal = data["zRightStepAmplitudeCal"];
                        msg.zLeftStepAmplitudeCal = data["zLeftStepAmplitudeCal"];
                        msg.maxRightStepAcceleration = data["maxRightStepAcceleration"];
                        msg.maxLeftStepAcceleration = data["maxLeftStepAcceleration"];
                        msg.stepCadence = data["stepCadence"];
                        msg.blinkIntervalMean = data["blinkIntervalMean"];
                        msg.blinkStrengthMean = data["blinkStrengthMean"];
                        msg.blinkStrengthSD = data["blinkStrengthSD"];
                        msg.blinkWidthMean = data["blinkWidthMean"];
                        msg.nptMean = data["nptMean"];
                        msg.nptSD = data["nptSD"];
                        msg.blinkCount = data["blinkCount"];
                        msg.blinkIntervalCount = data["blinkIntervalCount"];
                        msg.blinkIntervalTotal = data["blinkIntervalTotal"];
                        msg.blinkStrengthTotal = data["blinkStrengthTotal"];
                        msg.blinkStrengthMax = data["blinkStrengthMax"];
                        msg.nptMedian = data["nptMedian"];
                        msg.noiseTime = data["noiseTime"];
                        msg.isValid = data["isValid"];
                        msg.sleepScore = data["sleepScore"];
                        msg.focusScore = data["focusScore"];
                        msg.tensionScore = data["tensionScore"];
                        msg.calmScore = data["calmScore"];
                        msg.sleepScoreStandard = data["sleepScoreStandard"];
                        pub_logic_index_data.publish(msg);
                        rosnodejs.log.info("published.");
                    } else {
                        //メッセージを出力
                        console.info(data);
                    }
                } else {
                    rosnodejs.log.info("heatbeat: %s", message);
                }
            });
        });
    });
}

if (require.main === module) {
    main()
}
