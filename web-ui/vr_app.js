// 等待 A-Frame 场景加载

AFRAME.registerComponent('controller-updater', {
  init: function () {
    console.log("控制器更新组件已初始化。");
    // 控制器已启用

    this.leftHand = document.querySelector('#leftHand');
    this.rightHand = document.querySelector('#rightHand');
    this.leftHandInfoText = document.querySelector('#leftHandInfo');
    this.rightHandInfoText = document.querySelector('#rightHandInfo');

    // --- WebSocket 设置 ---
    this.websocket = null;
    this.leftGripDown = false;
    this.rightGripDown = false;
    this.leftTriggerDown = false;
    this.rightTriggerDown = false;

    // --- 状态报告 ---
    this.lastStatusUpdate = 0;
    this.statusUpdateInterval = 5000; // 5 seconds

    // --- 相对旋转跟踪 ---
    this.leftGripInitialRotation = null;
    this.rightGripInitialRotation = null;
    this.leftRelativeRotation = { x: 0, y: 0, z: 0 };
    this.rightRelativeRotation = { x: 0, y: 0, z: 0 };

    // --- 基于四元数的 Z 轴旋转跟踪 ---
    this.leftGripInitialQuaternion = null;
    this.rightGripInitialQuaternion = null;
    this.leftZAxisRotation = 0;
    this.rightZAxisRotation = 0;

    // --- 动态获取主机名 ---
    const serverHostname = window.location.hostname;
    const websocketPort = 8442; // 确保与 controller_server.py 中的端口一致
    const websocketUrl = `wss://${serverHostname}:${websocketPort}`;
    console.log(`尝试连接到 WebSocket: ${websocketUrl}`);
    // !!! 重要：将 'YOUR_LAPTOP_IP' 替换为您的笔记本电脑的实际 IP 地址 !!!
    // const websocketUrl = 'ws://YOUR_LAPTOP_IP:8442';
    try {
      this.websocket = new WebSocket(websocketUrl);
      this.websocket.onopen = (event) => {
        console.log(`WebSocket 已连接到 ${websocketUrl}`);
        this.reportVRStatus(true);
      };
      this.websocket.onerror = (event) => {
        // 更详细的错误日志
        console.error(`WebSocket 错误: 事件类型: ${event.type}`, event);
        this.reportVRStatus(false);
      };
      this.websocket.onclose = (event) => {
        console.log(`WebSocket 已从 ${websocketUrl} 断开连接。正常关闭: ${event.wasClean}, 代码: ${event.code}, 原因: '${event.reason}'`);
        // 如果可用，尝试记录具体错误（可能受浏览器安全限制）
        if (!event.wasClean) {
          console.error('WebSocket 意外关闭。');
        }
        this.websocket = null; // Clear the reference
        this.reportVRStatus(false);
      };
      this.websocket.onmessage = (event) => {
        console.log(`收到 WebSocket 消息: ${event.data}`); // 记录来自服务器的任何消息
      };
    } catch (error) {
        console.error(`无法创建到 ${websocketUrl} 的 WebSocket 连接:`, error);
        this.reportVRStatus(false);
    }
    // --- WebSocket 设置结束 ---

    // --- VR 状态报告函数 ---
    this.reportVRStatus = (connected) => {
      // 更新全局状态（如果可用，用于桌面界面）
      if (typeof updateStatus === 'function') {
        updateStatus({ vrConnected: connected });
      }
      
      // 如果在 iframe 中，也尝试通知父窗口
      try {
        if (window.parent && window.parent !== window) {
          window.parent.postMessage({
            type: 'vr_status',
            connected: connected
          }, '*');
        }
      } catch (e) {
        // 忽略跨域错误
      }
    };

    if (!this.leftHand || !this.rightHand || !this.leftHandInfoText || !this.rightHandInfoText) {
      console.error("未找到控制器或文本实体！");
      // 检查哪些特定元素缺失
      if (!this.leftHand) console.error("未找到左手实体");
      if (!this.rightHand) console.error("未找到右手实体");
      if (!this.leftHandInfoText) console.error("未找到左手信息文本");
      if (!this.rightHandInfoText) console.error("未找到右手信息文本");
      return;
    }

    // 为组合文本元素应用初始旋转
    const textRotation = '-90 0 0'; // 绕 X 轴旋转 -90 度
    if (this.leftHandInfoText) this.leftHandInfoText.setAttribute('rotation', textRotation);
    if (this.rightHandInfoText) this.rightHandInfoText.setAttribute('rotation', textRotation);

    // --- 创建坐标轴指示器 ---
    this.createAxisIndicators();

    // --- 辅助函数：发送握把释放消息 ---
    this.sendGripRelease = (hand) => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const releaseMessage = {
          hand: hand,
          gripReleased: true
        };
        this.websocket.send(JSON.stringify(releaseMessage));
        console.log(`发送${hand}手握把释放`);
      }
    };

    // --- 辅助函数：发送扳机释放消息 ---
    this.sendTriggerRelease = (hand) => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const releaseMessage = {
          hand: hand,
          triggerReleased: true
        };
        this.websocket.send(JSON.stringify(releaseMessage));
        console.log(`发送${hand}手扳机释放`);
      }
    };

    // --- 辅助函数：计算相对旋转 ---
    this.calculateRelativeRotation = (currentRotation, initialRotation) => {
      return {
        x: currentRotation.x - initialRotation.x,
        y: currentRotation.y - initialRotation.y,
        z: currentRotation.z - initialRotation.z
      };
    };

    // --- 辅助函数：从四元数计算 Z 轴旋转 ---
    this.calculateZAxisRotation = (currentQuaternion, initialQuaternion) => {
      // 计算相对四元数（从初始到当前）
      const relativeQuat = new THREE.Quaternion();
      relativeQuat.multiplyQuaternions(currentQuaternion, initialQuaternion.clone().invert());
      
      // 获取控制器的当前前进方向（世界空间中的局部 Z 轴）
      const forwardDirection = new THREE.Vector3(0, 0, 1);
      forwardDirection.applyQuaternion(currentQuaternion);
      
      // 将相对四元数转换为轴角表示
      const angle = 2 * Math.acos(Math.abs(relativeQuat.w));
      
      // 处理无旋转的情况（避免除以零）
      if (angle < 0.0001) {
        return 0;
      }
      
      // 获取旋转轴
      const sinHalfAngle = Math.sqrt(1 - relativeQuat.w * relativeQuat.w);
      const rotationAxis = new THREE.Vector3(
        relativeQuat.x / sinHalfAngle,
        relativeQuat.y / sinHalfAngle,
        relativeQuat.z / sinHalfAngle
      );
      
      // 将旋转轴投影到前进方向上，以获得围绕前进轴的旋转分量
      const projectedComponent = rotationAxis.dot(forwardDirection);
      
      // 围绕前进轴的旋转等于角度乘以投影
      const forwardRotation = angle * projectedComponent;
      
      // 转换为度数并正确处理符号
      let degrees = THREE.MathUtils.radToDeg(forwardRotation);
      
      // 归一化到 -180 到 +180 范围以避免突然跳变
      while (degrees > 180) degrees -= 360;
      while (degrees < -180) degrees += 360;
      
      return degrees;
    };

    // --- Modify Event Listeners ---
    this.leftHand.addEventListener('triggerdown', (evt) => {
        console.log('Left Trigger Pressed');
        this.leftTriggerDown = true;
    });
    this.leftHand.addEventListener('triggerup', (evt) => {
        console.log('Left Trigger Released');
        this.leftTriggerDown = false;
        this.sendTriggerRelease('left'); // Send trigger release message
    });
    this.leftHand.addEventListener('gripdown', (evt) => {
        console.log('左握把按下');
        this.leftGripDown = true; // 设置握把状态
        
        // 存储初始旋转以进行相对跟踪
        if (this.leftHand.object3D.visible) {
          const leftRotEuler = this.leftHand.object3D.rotation;
          this.leftGripInitialRotation = {
            x: THREE.MathUtils.radToDeg(leftRotEuler.x),
            y: THREE.MathUtils.radToDeg(leftRotEuler.y),
            z: THREE.MathUtils.radToDeg(leftRotEuler.z)
          };
          
          // 存储初始四元数以进行 Z 轴旋转跟踪
          this.leftGripInitialQuaternion = this.leftHand.object3D.quaternion.clone();
          
          console.log('左握把初始旋转:', this.leftGripInitialRotation);
          console.log('左握把初始四元数:', this.leftGripInitialQuaternion);
        }
    });
    this.leftHand.addEventListener('gripup', (evt) => { // 添加 gripup 监听器
        console.log('左握把释放');
        this.leftGripDown = false; // 重置握把状态
        this.leftGripInitialRotation = null; // 重置初始旋转
        this.leftGripInitialQuaternion = null; // 重置初始四元数
        this.leftRelativeRotation = { x: 0, y: 0, z: 0 }; // 重置相对旋转
        this.leftZAxisRotation = 0; // 重置 Z 轴旋转
        this.sendGripRelease('left'); // 发送握把释放消息
    });

    this.rightHand.addEventListener('triggerdown', (evt) => {
        console.log('右扳机按下');
        this.rightTriggerDown = true;
    });
    this.rightHand.addEventListener('triggerup', (evt) => {
        console.log('右扳机释放');
        this.rightTriggerDown = false;
        this.sendTriggerRelease('right'); // 发送扳机释放消息
    });
    this.rightHand.addEventListener('gripdown', (evt) => {
        console.log('右握把按下');
        this.rightGripDown = true; // 设置握把状态
        
        // 存储初始旋转以进行相对跟踪
        if (this.rightHand.object3D.visible) {
          const rightRotEuler = this.rightHand.object3D.rotation;
          this.rightGripInitialRotation = {
            x: THREE.MathUtils.radToDeg(rightRotEuler.x),
            y: THREE.MathUtils.radToDeg(rightRotEuler.y),
            z: THREE.MathUtils.radToDeg(rightRotEuler.z)
          };
          
          // 存储初始四元数以进行 Z 轴旋转跟踪
          this.rightGripInitialQuaternion = this.rightHand.object3D.quaternion.clone();
          
          console.log('右握把初始旋转:', this.rightGripInitialRotation);
          console.log('右握把初始四元数:', this.rightGripInitialQuaternion);
        }
    });
    this.rightHand.addEventListener('gripup', (evt) => { // 添加 gripup 监听器
        console.log('右握把释放');
        this.rightGripDown = false; // 重置握把状态
        this.rightGripInitialRotation = null; // 重置初始旋转
        this.rightGripInitialQuaternion = null; // 重置初始四元数
        this.rightRelativeRotation = { x: 0, y: 0, z: 0 }; // 重置相对旋转
        this.rightZAxisRotation = 0; // 重置 Z 轴旋转
        this.sendGripRelease('right'); // 发送握把释放消息
    });
    // --- 事件监听器修改结束 ---

  },

  createAxisIndicators: function() {
    // 为两个控制器创建 XYZ 坐标轴指示器
    
    // 左控制器坐标轴
    // X 轴（红色）
    const leftXAxis = document.createElement('a-cylinder');
    leftXAxis.setAttribute('id', 'leftXAxis');
    leftXAxis.setAttribute('height', '0.08');
    leftXAxis.setAttribute('radius', '0.003');
    leftXAxis.setAttribute('color', '#ff0000'); // X 轴用红色
    leftXAxis.setAttribute('position', '0.04 0 0');
    leftXAxis.setAttribute('rotation', '0 0 90'); // 旋转以沿 X 轴指向
    this.leftHand.appendChild(leftXAxis);

    const leftXTip = document.createElement('a-cone');
    leftXTip.setAttribute('height', '0.015');
    leftXTip.setAttribute('radius-bottom', '0.008');
    leftXTip.setAttribute('radius-top', '0');
    leftXTip.setAttribute('color', '#ff0000');
    leftXTip.setAttribute('position', '0.055 0 0');
    leftXTip.setAttribute('rotation', '0 0 90');
    this.leftHand.appendChild(leftXTip);

    // Y 轴（绿色）- 向上
    const leftYAxis = document.createElement('a-cylinder');
    leftYAxis.setAttribute('id', 'leftYAxis');
    leftYAxis.setAttribute('height', '0.08');
    leftYAxis.setAttribute('radius', '0.003');
    leftYAxis.setAttribute('color', '#00ff00'); // Y 轴用绿色
    leftYAxis.setAttribute('position', '0 0.04 0');
    leftYAxis.setAttribute('rotation', '0 0 0'); // 默认向上方向
    this.leftHand.appendChild(leftYAxis);

    const leftYTip = document.createElement('a-cone');
    leftYTip.setAttribute('height', '0.015');
    leftYTip.setAttribute('radius-bottom', '0.008');
    leftYTip.setAttribute('radius-top', '0');
    leftYTip.setAttribute('color', '#00ff00');
    leftYTip.setAttribute('position', '0 0.055 0');
    this.leftHand.appendChild(leftYTip);

    // Z 轴（蓝色）- 向前
    const leftZAxis = document.createElement('a-cylinder');
    leftZAxis.setAttribute('id', 'leftZAxis');
    leftZAxis.setAttribute('height', '0.08');
    leftZAxis.setAttribute('radius', '0.003');
    leftZAxis.setAttribute('color', '#0000ff'); // Z 轴用蓝色
    leftZAxis.setAttribute('position', '0 0 0.04');
    leftZAxis.setAttribute('rotation', '90 0 0'); // 旋转以沿 Z 轴指向
    this.leftHand.appendChild(leftZAxis);

    const leftZTip = document.createElement('a-cone');
    leftZTip.setAttribute('height', '0.015');
    leftZTip.setAttribute('radius-bottom', '0.008');
    leftZTip.setAttribute('radius-top', '0');
    leftZTip.setAttribute('color', '#0000ff');
    leftZTip.setAttribute('position', '0 0 0.055');
    leftZTip.setAttribute('rotation', '90 0 0');
    this.leftHand.appendChild(leftZTip);

    // 右控制器坐标轴
    // X 轴（红色）
    const rightXAxis = document.createElement('a-cylinder');
    rightXAxis.setAttribute('id', 'rightXAxis');
    rightXAxis.setAttribute('height', '0.08');
    rightXAxis.setAttribute('radius', '0.003');
    rightXAxis.setAttribute('color', '#ff0000'); // X 轴用红色
    rightXAxis.setAttribute('position', '0.04 0 0');
    rightXAxis.setAttribute('rotation', '0 0 90'); // 旋转以沿 X 轴指向
    this.rightHand.appendChild(rightXAxis);

    const rightXTip = document.createElement('a-cone');
    rightXTip.setAttribute('height', '0.015');
    rightXTip.setAttribute('radius-bottom', '0.008');
    rightXTip.setAttribute('radius-top', '0');
    rightXTip.setAttribute('color', '#ff0000');
    rightXTip.setAttribute('position', '0.055 0 0');
    rightXTip.setAttribute('rotation', '0 0 90');
    this.rightHand.appendChild(rightXTip);

    // Y 轴（绿色）- 向上
    const rightYAxis = document.createElement('a-cylinder');
    rightYAxis.setAttribute('id', 'rightYAxis');
    rightYAxis.setAttribute('height', '0.08');
    rightYAxis.setAttribute('radius', '0.003');
    rightYAxis.setAttribute('color', '#00ff00'); // Y 轴用绿色
    rightYAxis.setAttribute('position', '0 0.04 0');
    rightYAxis.setAttribute('rotation', '0 0 0'); // 默认向上方向
    this.rightHand.appendChild(rightYAxis);

    const rightYTip = document.createElement('a-cone');
    rightYTip.setAttribute('height', '0.015');
    rightYTip.setAttribute('radius-bottom', '0.008');
    rightYTip.setAttribute('radius-top', '0');
    rightYTip.setAttribute('color', '#00ff00');
    rightYTip.setAttribute('position', '0 0.055 0');
    this.rightHand.appendChild(rightYTip);

    // Z 轴（蓝色）- 向前
    const rightZAxis = document.createElement('a-cylinder');
    rightZAxis.setAttribute('id', 'rightZAxis');
    rightZAxis.setAttribute('height', '0.08');
    rightZAxis.setAttribute('radius', '0.003');
    rightZAxis.setAttribute('color', '#0000ff'); // Z 轴用蓝色
    rightZAxis.setAttribute('position', '0 0 0.04');
    rightZAxis.setAttribute('rotation', '90 0 0'); // 旋转以沿 Z 轴指向
    this.rightHand.appendChild(rightZAxis);

    const rightZTip = document.createElement('a-cone');
    rightZTip.setAttribute('height', '0.015');
    rightZTip.setAttribute('radius-bottom', '0.008');
    rightZTip.setAttribute('radius-top', '0');
    rightZTip.setAttribute('color', '#0000ff');
    rightZTip.setAttribute('position', '0 0 0.055');
    rightZTip.setAttribute('rotation', '90 0 0');
    this.rightHand.appendChild(rightZTip);

    console.log('已为两个控制器创建 XYZ 坐标轴指示器（RGB 对应 XYZ）');
  },

  tick: function () {
    // 如果控制器可见，更新控制器文本
    if (!this.leftHand || !this.rightHand) return; // 添加安全检查

    // --- 开始详细日志 ---
    if (this.leftHand.object3D) {
      // console.log(`Left Hand Raw - Visible: ${this.leftHand.object3D.visible}, Pos: ${this.leftHand.object3D.position.x.toFixed(2)},${this.leftHand.object3D.position.y.toFixed(2)},${this.leftHand.object3D.position.z.toFixed(2)}`);
    }
    if (this.rightHand.object3D) {
      // console.log(`Right Hand Raw - Visible: ${this.rightHand.object3D.visible}, Pos: ${this.rightHand.object3D.position.x.toFixed(2)},${this.rightHand.object3D.position.y.toFixed(2)},${this.rightHand.object3D.position.z.toFixed(2)}`);
    }
    // --- 结束详细日志 ---

    // 从两个控制器收集数据
    const leftController = {
        hand: 'left',
        position: null,
        rotation: null,
        gripActive: false,
        trigger: 0
    };
    
    const rightController = {
        hand: 'right',
        position: null,
        rotation: null,
        gripActive: false,
        trigger: 0
    };

    // 更新左手文本并收集数据
    if (this.leftHand.object3D.visible) {
        const leftPos = this.leftHand.object3D.position;
        const leftRotEuler = this.leftHand.object3D.rotation; // Euler angles in radians
        // Convert to degrees without offset
        const leftRotX = THREE.MathUtils.radToDeg(leftRotEuler.x);
        const leftRotY = THREE.MathUtils.radToDeg(leftRotEuler.y);
        const leftRotZ = THREE.MathUtils.radToDeg(leftRotEuler.z);

        // Calculate relative rotation if grip is held
        if (this.leftGripDown && this.leftGripInitialRotation) {
          this.leftRelativeRotation = this.calculateRelativeRotation(
            { x: leftRotX, y: leftRotY, z: leftRotZ },
            this.leftGripInitialRotation
          );
          
          // Calculate Z-axis rotation using quaternions
          if (this.leftGripInitialQuaternion) {
            this.leftZAxisRotation = this.calculateZAxisRotation(
              this.leftHand.object3D.quaternion,
              this.leftGripInitialQuaternion
            );
          }
          
          console.log('左相对旋转:', this.leftRelativeRotation);
          console.log('左 Z 轴旋转:', this.leftZAxisRotation.toFixed(1), '度');
        }

        // Create display text including relative rotation when grip is held
        let combinedLeftText = `Pos: ${leftPos.x.toFixed(2)} ${leftPos.y.toFixed(2)} ${leftPos.z.toFixed(2)}\\nRot: ${leftRotX.toFixed(0)} ${leftRotY.toFixed(0)} ${leftRotZ.toFixed(0)}`;
        if (this.leftGripDown && this.leftGripInitialRotation) {
          combinedLeftText += `\\nZ-Rot: ${this.leftZAxisRotation.toFixed(1)}°`;
        }

        if (this.leftHandInfoText) {
            this.leftHandInfoText.setAttribute('value', combinedLeftText);
        }

        // 收集左控制器数据
        leftController.position = { x: leftPos.x, y: leftPos.y, z: leftPos.z };
        leftController.rotation = { x: leftRotX, y: leftRotY, z: leftRotZ };
        leftController.quaternion = { 
          x: this.leftHand.object3D.quaternion.x, 
          y: this.leftHand.object3D.quaternion.y, 
          z: this.leftHand.object3D.quaternion.z, 
          w: this.leftHand.object3D.quaternion.w 
        };
        leftController.trigger = this.leftTriggerDown ? 1 : 0;
        leftController.gripActive = this.leftGripDown;
    }

    // 更新右手文本并收集数据
    if (this.rightHand.object3D.visible) {
        const rightPos = this.rightHand.object3D.position;
        const rightRotEuler = this.rightHand.object3D.rotation; // Euler angles in radians
        // Convert to degrees without offset
        const rightRotX = THREE.MathUtils.radToDeg(rightRotEuler.x);
        const rightRotY = THREE.MathUtils.radToDeg(rightRotEuler.y);
        const rightRotZ = THREE.MathUtils.radToDeg(rightRotEuler.z);

        // Calculate relative rotation if grip is held
        if (this.rightGripDown && this.rightGripInitialRotation) {
          this.rightRelativeRotation = this.calculateRelativeRotation(
            { x: rightRotX, y: rightRotY, z: rightRotZ },
            this.rightGripInitialRotation
          );
          
          // Calculate Z-axis rotation using quaternions
          if (this.rightGripInitialQuaternion) {
            this.rightZAxisRotation = this.calculateZAxisRotation(
              this.rightHand.object3D.quaternion,
              this.rightGripInitialQuaternion
            );
          }
          
          console.log('右相对旋转:', this.rightRelativeRotation);
          console.log('右 Z 轴旋转:', this.rightZAxisRotation.toFixed(1), '度');
        }

        // Create display text including relative rotation when grip is held
        let combinedRightText = `Pos: ${rightPos.x.toFixed(2)} ${rightPos.y.toFixed(2)} ${rightPos.z.toFixed(2)}\\nRot: ${rightRotX.toFixed(0)} ${rightRotY.toFixed(0)} ${rightRotZ.toFixed(0)}`;
        if (this.rightGripDown && this.rightGripInitialRotation) {
          combinedRightText += `\\nZ-Rot: ${this.rightZAxisRotation.toFixed(1)}°`;
        }

        if (this.rightHandInfoText) {
            this.rightHandInfoText.setAttribute('value', combinedRightText);
        }

        // 收集右控制器数据
        rightController.position = { x: rightPos.x, y: rightPos.y, z: rightPos.z };
        rightController.rotation = { x: rightRotX, y: rightRotY, z: rightRotZ };
        rightController.quaternion = { 
          x: this.rightHand.object3D.quaternion.x, 
          y: this.rightHand.object3D.quaternion.y, 
          z: this.rightHand.object3D.quaternion.z, 
          w: this.rightHand.object3D.quaternion.w 
        };
        rightController.trigger = this.rightTriggerDown ? 1 : 0;
        rightController.gripActive = this.rightGripDown;
    }

    // 如果 WebSocket 已打开且至少有一个控制器有有效数据，则发送组合数据包
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        const hasValidLeft = leftController.position && (leftController.position.x !== 0 || leftController.position.y !== 0 || leftController.position.z !== 0);
        const hasValidRight = rightController.position && (rightController.position.x !== 0 || rightController.position.y !== 0 || rightController.position.z !== 0);
        
        if (hasValidLeft || hasValidRight) {
            const dualControllerData = {
                timestamp: Date.now(),
                leftController: leftController,
                rightController: rightController
            };
            this.websocket.send(JSON.stringify(dualControllerData));
        }
    }
  }
});


// 场景加载后将组件添加到场景
document.addEventListener('DOMContentLoaded', (event) => {
    const scene = document.querySelector('a-scene');

    if (scene) {
        // Listen for controller connection events
        scene.addEventListener('controllerconnected', (evt) => {
            console.log('控制器已连接:', evt.detail.name, evt.detail.component.data.hand);
        });
        scene.addEventListener('controllerdisconnected', (evt) => {
            console.log('控制器已断开:', evt.detail.name, evt.detail.component.data.hand);
        });

        // 场景加载后添加 controller-updater 组件（A-Frame 管理会话）
        if (scene.hasLoaded) {
            scene.setAttribute('controller-updater', '');
            console.log("立即添加了 controller-updater 组件。");
        } else {
            scene.addEventListener('loaded', () => {
                scene.setAttribute('controller-updater', '');
                console.log("场景加载后添加了 controller-updater 组件。");
            });
        }
    } else {
        console.error('未找到 A-Frame 场景！');
    }

    // 添加控制器跟踪按钮逻辑
    addControllerTrackingButton();
});

function addControllerTrackingButton() {
    if (navigator.xr) {
        // 检查 immersive-ar（Quest 3/Pro）或 immersive-vr（Quest 2）支持
        Promise.all([
            navigator.xr.isSessionSupported('immersive-ar').catch(() => false),
            navigator.xr.isSessionSupported('immersive-vr').catch(() => false)
        ]).then(([arSupported, vrSupported]) => {
            if (arSupported || vrSupported) {
                // 创建开始控制器跟踪按钮
                const startButton = document.createElement('button');
                startButton.id = 'start-tracking-button';
                startButton.textContent = '开始控制器跟踪';
                startButton.style.position = 'fixed';
                startButton.style.top = '50%';
                startButton.style.left = '50%';
                startButton.style.transform = 'translate(-50%, -50%)';
                startButton.style.padding = '20px 40px';
                startButton.style.fontSize = '20px';
                startButton.style.fontWeight = 'bold';
                startButton.style.backgroundColor = '#4CAF50';
                startButton.style.color = 'white';
                startButton.style.border = 'none';
                startButton.style.borderRadius = '8px';
                startButton.style.cursor = 'pointer';
                startButton.style.zIndex = '9999';
                startButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
                startButton.style.transition = 'all 0.3s ease';

                // 悬停效果
                startButton.addEventListener('mouseenter', () => {
                    startButton.style.backgroundColor = '#45a049';
                    startButton.style.transform = 'translate(-50%, -50%) scale(1.05)';
                });
                startButton.addEventListener('mouseleave', () => {
                    startButton.style.backgroundColor = '#4CAF50';
                    startButton.style.transform = 'translate(-50%, -50%) scale(1)';
                });

                startButton.onclick = async () => {
                    console.log('点击了开始控制器跟踪按钮。');
                    const sceneEl = document.querySelector('a-scene');
                    if (!sceneEl) {
                        console.error('未找到用于 enterVR 调用的 A-Frame 场景！');
                        return;
                    }

                    // 更新按钮以显示正在连接
                    startButton.textContent = '连接中...';
                    startButton.disabled = true;

                    try {
                        // 检查机器人是否已连接
                        const statusResponse = await fetch('/api/status');
                        const status = await statusResponse.json();

                        if (!status.robotEngaged) {
                            console.log('机器人未连接。先连接机械臂...');
                            startButton.textContent = '连接机械臂中...';

                            // 连接机器人机械臂
                            const connectResponse = await fetch('/api/robot', {
                                method: 'POST',
                                headers: {
                                    'Content-Type': 'application/json'
                                },
                                body: JSON.stringify({ action: 'connect' })
                            });
                            const connectResult = await connectResponse.json();

                            if (!connectResult.success) {
                                throw new Error(connectResult.error || '无法连接机器人机械臂');
                            }
                            console.log('机器人机械臂连接成功。');

                            // 等待片刻让机械臂初始化
                            await new Promise(resolve => setTimeout(resolve, 500));
                        } else {
                            console.log('机器人已连接。');
                        }

                        // 现在进入 VR 模式
                        console.log('通过 A-Frame 请求 VR 会话...');
                        startButton.textContent = '启动 VR 中...';
                        await sceneEl.enterVR(true);
                    } catch (err) {
                        console.error('无法启动控制器跟踪:', err);
                        alert(`启动失败: ${err.message}`);
                        // 重置按钮状态
                        startButton.textContent = '开始控制器跟踪';
                        startButton.disabled = false;
                    }
                };

                document.body.appendChild(startButton);
                console.log('已添加官方"开始控制器跟踪"按钮。');

                // 添加 VR 说明面板
                createVrInstructionsPanel();

                // 显示返回桌面按钮（函数在 interface.js 中定义）
                if (typeof showBackToDesktopButton === 'function') {
                    showBackToDesktopButton();
                }

                // 监听 VR 会话事件以隐藏/显示开始按钮
                const sceneEl = document.querySelector('a-scene');
                if (sceneEl) {
                    sceneEl.addEventListener('enter-vr', () => {
                        console.log('已进入 VR - 隐藏开始按钮');
                        startButton.style.display = 'none';
                    });

                    sceneEl.addEventListener('exit-vr', () => {
                        console.log('已退出 VR - 显示开始按钮');
                        startButton.style.display = 'block';
                    });
                }

            } else {
                console.warn('此浏览器/设备不支持 immersive-ar 或 immersive-vr。');
            }
        }).catch((err) => {
            console.error('检查 XR 支持时出错:', err);
        });
    } else {
        console.warn('此浏览器不支持 WebXR。');
    }
}

function createVrInstructionsPanel() {
    // 如果已存在则不创建
    if (document.getElementById('vr-instructions-panel')) return;

    const panel = document.createElement('div');
    panel.id = 'vr-instructions-panel';
    panel.style.cssText = `
        position: fixed;
        top: 20px;
        left: 50%;
        transform: translateX(-50%);
        max-width: 90%;
        width: 600px;
        background: rgba(15, 52, 96, 0.95);
        border-radius: 12px;
        padding: 20px;
        color: white;
        z-index: 9998;
        box-shadow: 0 8px 32px rgba(0,0,0,0.3);
        border: 1px solid rgba(255,255,255,0.1);
    `;

    panel.innerHTML = `
        <h2 style="margin: 0 0 15px 0; font-size: 1.2em; text-align: center;">VR 控制器使用说明</h2>
        <div style="display: flex; gap: 15px; align-items: flex-start; flex-wrap: wrap;">
            <div style="flex: 1; min-width: 150px; text-align: center;">
                <img src="media/telegrip_instructions.jpg" alt="VR 控制器使用说明"
                     style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.2);">
            </div>
            <div style="flex: 1; min-width: 200px; display: flex; flex-direction: column; gap: 8px; font-size: 14px;">
                <div style="padding: 8px; background: rgba(255,255,255,0.1); border-radius: 6px;">
                    <strong style="color: #ee4d9a;">握把按钮：</strong>按住以移动机械臂
                </div>
                <div style="padding: 8px; background: rgba(255,255,255,0.1); border-radius: 6px;">
                    <strong style="color: #9af58c;">扳机：</strong>按住以闭合夹爪
                </div>
            </div>
        </div>
    `;

    document.body.appendChild(panel);

    // 进入 VR 时隐藏面板
    const sceneEl = document.querySelector('a-scene');
    if (sceneEl) {
        sceneEl.addEventListener('enter-vr', () => {
            panel.style.display = 'none';
        });
        sceneEl.addEventListener('exit-vr', () => {
            panel.style.display = 'block';
        });
    }
} 