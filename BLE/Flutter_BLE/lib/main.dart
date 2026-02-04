import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';
// Asegúrate de que esta importación sea correcta para tu BleService
import 'package:bluetooth_test_clean/services/ble_service.dart'; 

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'BLE Simple Test',
      home: const BluetoothTestScreen(),
    );
  }
}

class BluetoothTestScreen extends StatefulWidget {
  const BluetoothTestScreen({super.key});

  @override
  State<BluetoothTestScreen> createState() => _BluetoothTestScreenState();
}

class _BluetoothTestScreenState extends State<BluetoothTestScreen> {
  final BleService _bleService = BleService();
  String _status = "Inicializando...";

  @override
  void initState() {
    super.initState();
    /// _startProcess(); 
    // Ahora el estado inicial será "Listo para conectar" hasta que pulses el botón.
    setState(() {
      _status = "Listo para conectar manualmente.";
    });
  }

  Future<void> _startProcess() async {
    setState(() => _status = "Solicitando permisos...");
    
    // Solicitar todos los permisos necesarios
    Map<Permission, PermissionStatus> statuses = await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.location,
    ].request();

    if (statuses[Permission.bluetoothScan]!.isGranted && 
        statuses[Permission.bluetoothConnect]!.isGranted) {
      setState(() => _status = "Permisos OK. Conectando...");
      try {
        await _bleService.connectToESP32();
        setState(() => _status = "✅ Conectado exitosamente.");
      } catch (e) {
        setState(() => _status = "❌ Falló la conexión: ${e.toString()}");
      }
    } else {
      setState(() => _status = "❌ Permisos denegados.");
    }
  }

  @override
  void dispose() {
    _bleService.disconnect();
   // _bleService.dispose(); // Asegúrate de cerrar los streams
    super.dispose();
  }

  // Funciones de control de LEDs
  void _turnOnLedRojo() {
    // Envía una lista de bytes que contiene solo el valor 2 (Encender Rojo)
    _bleService.writeBytes([2]); 
  }

  void _turnOffLedRojo() {
    // Envía una lista de bytes que contiene solo el valor 1 (Apagar Rojo)
    _bleService.writeBytes([1]); 
  }

  void _turnOnLedVerde() {
    // Envía una lista de bytes que contiene solo el valor 4 (Encender Verde)
    _bleService.writeBytes([4]); 
  }

  void _turnOffLedVerde() {
    // Envía una lista de bytes que contiene solo el valor 3 (Apagar Verde)
    _bleService.writeBytes([3]); 
  }

  // Añade esta función para refrescar manualmente
  Future<void> _refreshTemperatures() async {
    // Llama a las funciones internas de lectura de BleService
    if (_bleService.device != null) {
      // Necesitamos añadir esta función a BleService
      await _bleService.readAllTemperatures(); 
      setState(() {
        _status = "Temperaturas actualizadas manualmente.";
      }); // <-- ERROR CORREGIDO AQUÍ
    }
  }

  @override // <-- Este override ahora está dentro de la clase _BluetoothTestScreenState
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Transformadores Distribucion ')),
      
      body: SingleChildScrollView( // Añadido SingleChildScrollView por si la pantalla es pequeña
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              // --- Estado de Conexión ---
              Image.asset(
              'assets/logo.png', // Usa la ruta de tu archivo
              height: 100,      // Ajusta el alto del logo a tu gusto
              width: 100,       // Ajusta el ancho del logo a tu gusto
            ),
              Card(
                margin: EdgeInsets.only(bottom: 20),
                child: Column( 
                  children: [
                    Text(
                      _status,
                      textAlign: TextAlign.center,
                      style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: _status.contains("✅") ? Colors.green : Colors.red),
                    ),
                    SizedBox(height: 15), // Añade un espacio
                    // 👇👇 AGREGAMOS EL BOTÓN MANUAL AQUÍ 👇👇
                    ElevatedButton(
                      onPressed: _startProcess, // Llama a la función de conexión
                      child: Text("Conectar a ESP32 Manualmente"),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.deepPurple, 
                        foregroundColor: Colors.white,
                        minimumSize: Size.fromHeight(40)
                      ),
                    ),
                  ],
                ),
              ),
            
              
              // --- Lectura de Temperaturas (StreamBuilder) ---
              Card(
                elevation: 4.0,
                margin: EdgeInsets.only(bottom: 20),
                child: Padding(
                  padding: const EdgeInsets.all(16.0),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text("Monitoreo en Tiempo Real", style: Theme.of(context).textTheme.titleMedium),
                      SizedBox(height: 10),
                      StreamBuilder<double>(
                        stream: _bleService.temp1Stream,
                        initialData: 0.0,
                        builder: (context, snapshot) {
                          return Text(
                            'Temp 1 (0xFF01): ${snapshot.data?.toStringAsFixed(2) ?? "N/A"} °C',
                            style: TextStyle(fontSize: 22, fontWeight: FontWeight.bold),
                          );
                        },
                      ),
                      SizedBox(height: 10),
                      StreamBuilder<double>(
                        stream: _bleService.temp2Stream,
                        initialData: 0.0,
                        builder: (context, snapshot) {
                          return Text(
                            'Temp 2 (0xFF02): ${snapshot.data?.toStringAsFixed(2) ?? "N/A"} °C',
                            style: TextStyle(fontSize: 22, fontWeight: FontWeight.bold),
                          );
                        },
                      ),
                    ],
                  ),
                ),
              ),
              
              // --- Botón de Refrescar Manualmente ---
              ElevatedButton(
                onPressed: _refreshTemperatures,
                child: Text("Refrescar Temperaturas Manualmente"),
                style: ElevatedButton.styleFrom(backgroundColor: Colors.blue, minimumSize: Size.fromHeight(40)),
              ),
              
              SizedBox(height: 30),

              // --- Control de LEDs Rojos ---
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(16.0),
                  child: Column(
                    children: [
                      Text("Control RELE 1", style: TextStyle(fontWeight: FontWeight.bold)),
                      SizedBox(height: 10),
                      Row(
                        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                        children: [
                          ElevatedButton(
                            onPressed: _turnOnLedRojo,
                            child: Text("Encender RELE 1"),
                            style: ElevatedButton.styleFrom(backgroundColor: Colors.green),
                          ),
                          ElevatedButton(
                            onPressed: _turnOffLedRojo,
                            child: Text("Apagar RELE 1"),
                            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
              ),

              SizedBox(height: 10),
              
              // --- Control de LEDs Verdes ---
              Card(
                child: Padding(
                  padding: const EdgeInsets.all(16.0),
                  child: Column(
                    children: [
                      Text("Control RELE 2", style: TextStyle(fontWeight: FontWeight.bold)),
                      SizedBox(height: 10),
                      Row(
                        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                        children: [
                          ElevatedButton(
                            onPressed: _turnOnLedVerde,
                            child: Text("Encender RELE 2"),
                            style: ElevatedButton.styleFrom(backgroundColor: Colors.green),
                          ),
                          ElevatedButton(
                            onPressed: _turnOffLedVerde,
                            child: Text("Apagar RELE 2"),
                            style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
                         ), // <-- Llave faltante 1 (Cierra ElevatedButton)
                        ],
                      ), // <-- Llave faltante 2 (Cierra Row)
                    ],
                  ), // <-- Llave faltante 3 (Cierra Column)
                ),
              ), // <-- Llave faltante 4 (Cierra Card)
            ],
          ), // <-- Llave faltante 5 (Cierra Column principal)
        ),
      ), // <-- Llave faltante 6 (Cierra SingleChildScrollView)
    );
  } // <-- Cierra Widget build

} // <-- Cierra clase _BluetoothTestScreenState
