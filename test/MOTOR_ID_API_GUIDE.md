# 电机ID设置 API 使用指南

## 功能说明

通过 WebSocket API 设置总线舵机的ID。

**重要**: 这是一个**纯硬件操作**，与业务逻辑（左臂/右臂）无关。只需提供串口、舵机类型和ID即可。

## API 端点

- **类型**: WebSocket 命令
- **命令**: `set_motor_id`

## 请求参数

```typescript
interface SetMotorIdCommand {
  action: 'set_motor_id';
  port: string;              // 串口号 (如 "COM3", "/dev/ttyUSB0")
  servo_type: string;        // 舵机类型 ("lx16a" 或 "st3215")
  old_id: number;            // 当前ID (1-253)
  new_id: number;            // 新ID (1-253)
  baudrate?: number;         // 波特率 (可选，默认115200)
}
```

## 前端调用示例

### JavaScript/TypeScript

```javascript
// 通过 WebSocket 发送命令
const setMotorId = async (port, servoType, oldId, newId, baudrate = 115200) => {
  const command = {
    action: 'set_motor_id',
    port: port,              // 串口号
    servo_type: servoType,   // 舵机类型
    old_id: oldId,           // 当前ID
    new_id: newId,           // 新ID
    baudrate: baudrate       // 波特率（可选）
  };
  
  // 假设 ws 是已连接的 WebSocket 实例
  ws.send(JSON.stringify(command));
  
  return new Promise((resolve, reject) => {
    // 监听响应（需要根据实际实现调整）
    const handler = (event) => {
      const response = JSON.parse(event.data);
      if (response.action === 'set_motor_id_response') {
        ws.removeEventListener('message', handler);
        resolve(response.success);
      }
    };
    
    ws.addEventListener('message', handler);
    
    // 超时处理
    setTimeout(() => {
      ws.removeEventListener('message', handler);
      reject(new Error('设置电机ID超时'));
    }, 5000);
  });
};

// 使用示例
try {
  const success = await setMotorId('COM3', 'lx16a', 1, 10);
  if (success) {
    console.log('✅ ID设置成功');
  } else {
    console.error('❌ ID设置失败');
  }
} catch (error) {
  console.error('设置ID异常:', error);
}
```

### Vue 组件示例

```vue
<template>
  <div class="motor-id-settings">
    <h3>舵机ID设置</h3>
    
    <div class="form-group">
      <label>串口号:</label>
      <input 
        type="text" 
        v-model="port" 
        placeholder="COM3 或 /dev/ttyUSB0"
      />
    </div>
    
    <div class="form-group">
      <label>舵机类型:</label>
      <select v-model="servoType">
        <option value="lx16a">LX-16A (幻尔科技)</option>
        <option value="st3215">ST3215 (飞特科技)</option>
      </select>
    </div>
    
    <div class="form-group">
      <label>当前ID:</label>
      <input 
        type="number" 
        v-model.number="oldId" 
        min="1" 
        max="253"
        placeholder="输入当前ID"
      />
    </div>
    
    <div class="form-group">
      <label>新ID:</label>
      <input 
        type="number" 
        v-model.number="newId" 
        min="1" 
        max="253"
        placeholder="输入新ID"
      />
    </div>
    
    <button @click="handleSetId" :disabled="isSetting">
      {{ isSetting ? '设置中...' : '设置ID' }}
    </button>
    
    <div v-if="message" :class="['message', messageType]">
      {{ message }}
    </div>
  </div>
</template>

<script>
export default {
  name: 'MotorIdSettings',
  data() {
    return {
      port: 'COM3',
      servoType: 'lx16a',
      oldId: 1,
      newId: null,
      isSetting: false,
      message: '',
      messageType: '' // 'success' | 'error'
    };
  },
  methods: {
    async handleSetId() {
      if (!this.port) {
        this.showMessage('请输入串口号', 'error');
        return;
      }
      
      if (!this.oldId || !this.newId) {
        this.showMessage('请输入当前ID和新ID', 'error');
        return;
      }
      
      if (this.newId < 1 || this.newId > 253) {
        this.showMessage('新ID必须在 1-253 范围内', 'error');
        return;
      }
      
      this.isSetting = true;
      this.message = '';
      
      try {
        const success = await this.setMotorId(
          this.port,
          this.servoType,
          this.oldId,
          this.newId
        );
        
        if (success) {
          this.showMessage(`✅ ID已从 ${this.oldId} 设置为 ${this.newId}`, 'success');
          // 更新当前ID
          this.oldId = this.newId;
        } else {
          this.showMessage('❌ ID设置失败，请检查连接和参数', 'error');
        }
      } catch (error) {
        this.showMessage(`❌ 设置异常: ${error.message}`, 'error');
      } finally {
        this.isSetting = false;
      }
    },
    
    async setMotorId(port, servoType, oldId, newId) {
      return new Promise((resolve, reject) => {
        const command = {
          action: 'set_motor_id',
          port,
          servo_type: servoType,
          old_id: oldId,
          new_id: newId
        };
        
        // 发送到后端
        this.$ws.send(JSON.stringify(command));
        
        // 监听响应
        const handler = (event) => {
          const response = JSON.parse(event.data);
          if (response.type === 'motor_id_result') {
            this.$ws.removeEventListener('message', handler);
            resolve(response.success);
          }
        };
        
        this.$ws.addEventListener('message', handler);
        
        // 超时
        setTimeout(() => {
          this.$ws.removeEventListener('message', handler);
          reject(new Error('操作超时'));
        }, 5000);
      });
    },
    
    showMessage(text, type) {
      this.message = text;
      this.messageType = type;
      setTimeout(() => {
        this.message = '';
      }, 3000);
    }
  }
};
</script>

<style scoped>
.motor-id-settings {
  padding: 20px;
  max-width: 400px;
}

.form-group {
  margin-bottom: 15px;
}

.form-group label {
  display: block;
  margin-bottom: 5px;
  font-weight: bold;
}

.form-group select,
.form-group input {
  width: 100%;
  padding: 8px;
  border: 1px solid #ddd;
  border-radius: 4px;
}

button {
  width: 100%;
  padding: 10px;
  background-color: #4CAF50;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
}

button:disabled {
  background-color: #ccc;
  cursor: not-allowed;
}

.message {
  margin-top: 15px;
  padding: 10px;
  border-radius: 4px;
}

.message.success {
  background-color: #d4edda;
  color: #155724;
}

.message.error {
  background-color: #f8d7da;
  color: #721c24;
}
</style>
```

## 注意事项

1. **ID范围**: 1-253
2. **唯一性**: 同一总线上的每个舵机必须有唯一的ID
3. **断电重启**: 设置ID后需要断电重启才能生效（某些舵机可能需要）
4. **串口占用**: 设置ID时会临时打开串口，操作完成后自动关闭
5. **安全提示**: 设置ID时建议先失能力矩，避免意外运动
6. **与业务无关**: 此功能不涉及机械臂、关节等业务概念，只是纯硬件操作

## 错误处理

常见错误及解决方案：

| 错误信息 | 原因 | 解决方案 |
|---------|------|----------|
| ID超出范围 | old_id 或 new_id 不在 1-253 范围内 | 检查ID值 |
| 不支持的舵机类型 | servo_type 不是 'lx16a' 或 'st3215' | 使用正确的舵机类型 |
| 无法连接到串口 | 串口号错误或端口被占用 | 检查串口号，关闭其他占用程序 |
| ID设置失败 | 通信错误或ID冲突 | 检查接线、确认旧ID正确 |
| 操作超时 | 通信延迟或无响应 | 检查硬件连接

## 后端日志示例

```
INFO: 🔧 设置电机ID: COM3 (lx16a) ID 1 → 10
INFO: 📦 创建 LX-16A 驱动 (端口: COM3, 波特率: 115200)
INFO: ✅ 串口 COM3 连接成功
INFO: ✅ 电机ID设置成功: COM3 ID 1 → 10
INFO: 🔌 串口 COM3 已断开
INFO: ✅ 电机ID设置成功: COM3 1 → 10
```

## 完整工作流程

1. 前端输入串口、舵机类型、旧ID和新ID
2. 点击“设置ID”按钮
3. 后端通过 WebSocket 接收命令
4. 调用 `motor_controller.set_motor_id()`
5. 根据舵机类型创建临时驱动实例
6. 连接串口并执行 ID 设置
7. 断开串口连接
8. 返回结果给前端
9. 前端显示成功/失败消息
