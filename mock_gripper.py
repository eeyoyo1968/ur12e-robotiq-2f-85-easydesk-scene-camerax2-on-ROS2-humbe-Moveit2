from pymodbus.server.sync import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext

def run_gripper_server():
    # 0x0900 is the 'Object Detection' + 'Activated' status for Robotiq
    # We populate the registers so the URCap sees a 'Ready' state.
    store = ModbusSlaveContext(
        hr=ModbusSequentialDataBlock(0, [0x0900] * 100)
    )
    context = ModbusServerContext(slaves=store, single=True)

    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Robotiq'
    identity.ProductCode = '2F-85'
    identity.ProductName = 'Mock Gripper'

    print("Starting Mock Robotiq Server on 0.0.0.0:502...")
    print("Press Ctrl+C to stop.")
    
    # Run the synchronous server
    StartTcpServer(context=context, identity=identity, address=("0.0.0.0", 502))

if __name__ == "__main__":
    try:
        run_gripper_server()
    except Exception as e:
        print(f"Error: {e}")