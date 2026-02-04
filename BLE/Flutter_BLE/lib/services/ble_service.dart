import 'dart:async';
import 'dart:typed_data';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

class BleService {
  BluetoothDevice? device;
  // targetCharacteristic ya no se usa, usamos _ledControlChar, etc.
  // BluetoothCharacteristic? targetCharacteristic; 
  BluetoothCharacteristic? _tempChar1;
  BluetoothCharacteristic? _tempChar2;
  BluetoothCharacteristic? _ledControlChar; 
  
  // *** Asegúrate que este nombre sea EXACTO ***
  final String DEVICE_NAME = "ESP_GATTS_DEMO"; 
  final String SERVICE_UUID = "000000ff-0000-1000-8000-00805f9b34fb";
  final String LED_CONTROL_UUID = "0000ff03-0000-1000-8000-00805f9b34fb";
  final String TEMP_CHAR_1_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"; 
  final String TEMP_CHAR_2_UUID = "0000ff02-0000-1000-8000-00805f9b34fb"; 
 
  // StreamControllers DEBEN inicializarse aquí, no dentro de connectToESP32
  final _temp1Controller = StreamController<double>.broadcast();
  final _temp2Controller = StreamController<double>.broadcast();
  Stream<double> get temp1Stream => _temp1Controller.stream;
  Stream<double> get temp2Stream => _temp2Controller.stream;


  // Función auxiliar para normalizar UUIDs
  String _normalizeUuid(String uuid) {
    if (uuid.length == 4) {
      return "0000$uuid-0000-1000-8000-00805f9b34fb".toUpperCase();
    }
    return uuid.toUpperCase();
  }
// Función pública para forzar la lectura de ambas temperaturas
  Future<void> readAllTemperatures() async {
    if (_tempChar1 != null && _tempChar2 != null) {
      await _readTemperature(_tempChar1!, _temp1Controller);
      await _readTemperature(_tempChar2!, _temp2Controller);
    } else {
      print("Error: Características de temperatura no disponibles para lectura manual.");
    }
  }

  Future<void> connectToESP32() async {
    if (!await FlutterBluePlus.isOn) {
      throw Exception("Bluetooth no está activado.");
    }

    // --- Lógica de Escaneo ---
    await FlutterBluePlus.startScan(timeout: Duration(seconds: 10));
    var subscription = FlutterBluePlus.scanResults.listen((results) {
      for (ScanResult r in results) {
        if (r.device.platformName == DEVICE_NAME) {
          device = r.device;
          FlutterBluePlus.stopScan();
          break;
        }
      }
    });
    await FlutterBluePlus.isScanning.where((val) => val == false).first;
    await subscription.cancel();

    if (device == null) {
      throw Exception("No se encontró el ESP32 con nombre $DEVICE_NAME.");
    }

    // --- Lógica de Conexión ---
    await device!.connect(
    license: License.free, // 'license' ya no es necesario en versiones recientes de fbp
    timeout: Duration(seconds: 10),
    autoConnect: false,
    );
    
    // --- Lógica de Descubrimiento de Servicios y Características ---
    List<BluetoothService> services = await device!.discoverServices(subscribeToServicesChanged: false);
    
    bool foundService = false;
    for (var service in services) {
      String serviceUuidNormalized = _normalizeUuid(service.uuid.toString());

      if (serviceUuidNormalized == SERVICE_UUID.toUpperCase()) {
        print("🎉🎉🎉 ¡Coincidencia de Servicio 0x00FF! 🎉🎉🎉");
        foundService = true;

        for (var characteristic in service.characteristics) {
          String charUuidNormalized = _normalizeUuid(characteristic.uuid.toString());
          print("  - UUID de característica (Normalizado): $charUuidNormalized");

          if (charUuidNormalized == _normalizeUuid(TEMP_CHAR_1_UUID)) {
            _tempChar1 = characteristic;
            await _readTemperature(_tempChar1!, _temp1Controller);
          } else if (charUuidNormalized == _normalizeUuid(TEMP_CHAR_2_UUID)) {
            _tempChar2 = characteristic;
             await _readTemperature(_tempChar2!, _temp2Controller);
          } else if (charUuidNormalized == _normalizeUuid(LED_CONTROL_UUID)) {
            _ledControlChar = characteristic;
            // targetCharacteristic = characteristic; // Opcional, ya no se usa
          }
        }
      }
    }
    
    if (!foundService || _ledControlChar == null || _tempChar1 == null || _tempChar2 == null) {
        throw Exception("No se encontraron todos los servicios/características GATT esperados.");
    }
    
    print("Conexión y descubrimiento completados con éxito.");
  }

  Future<void> disconnect() async {
    if (device != null) {
      await device!.disconnect();
      device = null;
      // Importante cerrar los streams cuando se desconecta
     _temp1Controller.close();
     _temp2Controller.close();
    }
  }

  // Función para leer y decodificar el flotante de 32 bits (Little Endian)
  Future<void> _readTemperature(BluetoothCharacteristic char, StreamController<double> controller) async {
    List<int> bytes = await char.read();
    
    final buffer = Uint8List.fromList(bytes).buffer;
    final view = ByteData.view(buffer);
    final floatValue = view.getFloat32(0, Endian.little); 

    controller.add(floatValue); // Envía el valor al Stream
  }
 
  // Funcion enviar Hexa  
  Future<void> writeBytes(List<int> bytes) async {
    if (_ledControlChar == null) {
        print("Error: Característica de control LED no disponible.");
        return;
    }
    await _ledControlChar!.write(bytes, withoutResponse: false); 
    print("Datos (bytes) enviados: $bytes");
  }

}

