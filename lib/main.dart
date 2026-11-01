import 'dart:async';
import 'package:flutter/material.dart';
import 'package:camera/camera.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'native_draft_bridge.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final cameras = await availableCameras();
  runApp(MaterialApp(
    debugShowCheckedModeBanner: false,
    theme: ThemeData.dark(useMaterial3: true),
    home: DraftSurveyScreen(cameras: cameras),
  ));
}

class SeedPoint {
  final Offset imagePx;
  final Offset screenPx;
  final double zMeter;
  SeedPoint({required this.imagePx, required this.screenPx, required this.zMeter});
}

class DraftSurveyScreen extends StatefulWidget {
  final List<CameraDescription> cameras;
  const DraftSurveyScreen({super.key, required this.cameras});

  @override
  State<DraftSurveyScreen> createState() => _DraftSurveyScreenState();
}

class _DraftSurveyScreenState extends State<DraftSurveyScreen> {
  CameraController? _camController;
  NativeDraftBridge? _bridge;
  StreamSubscription<AccelerometerEvent>? _imuSub;

  double _gx = 0.0, _gy = 9.81, _gz = 0.0;
  bool _isStreaming = false;
  bool _isLockedOn = false;
  String _selectedStation = 'FWD Port (Flare 32°)';
  double _nextSeedDraftM = 9.0; // 터치 시 부여할 기본 흘수 마크 값 (0.4m씩 자동 감소)

  final List<SeedPoint> _manualSeeds = [];
  final List<double> _waveHistory = [];
  CameraImage? _latestImage;

  // 실시간 AR 렌더링 상태
  double _instantDraftM = 0.0;
  Offset _waterlinePx = Offset.zero;
  List<Offset> _spineCurvePx = [];
  List<MapEntry<double, Offset>> _trackedMarks = [];
  int _collectedFrames = 0;

  @override
  void initState() {
    super.initState();
    _bridge = NativeDraftBridge(fps: 30.0, flareDeg: 32.0);
    _initSensors();
    _initCamera();
  }

  void _initSensors() {
    _imuSub = accelerometerEventStream().listen((event) {
      // 저역통과 필터로 선형 파도 충격 가속도 완화 및 중력 벡터 추출
      _gx = 0.85 * _gx + 0.15 * event.x;
      _gy = 0.85 * _gy + 0.15 * event.y;
      _gz = 0.85 * _gz + 0.15 * event.z;
    });
  }

  Future<void> _initCamera() async {
    if (widget.cameras.isEmpty) return;
    _camController = CameraController(
      widget.cameras.first,
      ResolutionPreset.veryHigh, // 1080p 고정
      enableAudio: false,
      imageFormatGroup: ImageFormatGroup.yuv420,
    );
    await _camController!.initialize();
    await _camController!.startImageStream(_onCameraFrame);
    if (mounted) setState(() {});
  }

  void _onCameraFrame(CameraImage img) {
    _latestImage = img;
    if (!_isLockedOn || !_isStreaming || _bridge == null) return;

    final yPlane = img.planes[0];
    final tel = _bridge!.processFrame(
      yPlane.bytes,
      img.width,
      img.height,
      yPlane.bytesPerRow,
      _gx,
      _gy,
      _gz,
    );

    if (tel.statusCode == 1) {
      final marks = <MapEntry<double, Offset>>[];
      for (int i = 0; i < tel.activeMarkCount; i++) {
        marks.add(MapEntry(tel.markZ[i], Offset(tel.markU[i], tel.markV[i])));
      }
      final spine = <Offset>[];
      for (int i = 0; i < tel.spinePtsCount; i++) {
        spine.add(Offset(tel.spineU[i], tel.spineV[i]));
      }

      setState(() {
        _instantDraftM = tel.instantDraftM;
        _waterlinePx = Offset(tel.waterlineU, tel.waterlineV);
        _trackedMarks = marks;
        _spineCurvePx = spine;
        _collectedFrames = tel.collectedFrames;
        _waveHistory.add(tel.instantDraftM);
        if (_waveHistory.length > 300) _waveHistory.removeAt(0);
      });

      // 40초(1200프레임) 수집 달성 시 자동 완료 및 리포트 출력
      if (_collectedFrames >= 1200) {
        _finishAndShowReport();
      }
    }
  }

  // 사용자가 화면의 마크 밑단을 터치하여 부트스트랩 시드 추가
  void _onTapViewfinder(TapUpDetails details, BoxConstraints box) {
    if (_isLockedOn || _latestImage == null) return;

    final imgW = _latestImage!.width.toDouble();
    final imgH = _latestImage!.height.toDouble();
    final u = (details.localPosition.dx / box.maxWidth) * imgW;
    final v = (details.localPosition.dy / box.maxHeight) * imgH;

    setState(() {
      _manualSeeds.add(SeedPoint(
        imagePx: Offset(u, v),
        screenPx: details.localPosition,
        zMeter: _nextSeedDraftM,
      ));
      // 다음 터치 마크는 0.4m 아래 마크로 자동 프리셋 (예: 9.0 -> 8.6 -> 8.2)
      _nextSeedDraftM = double.parse((_nextSeedDraftM - 0.4).toStringAsFixed(1));
    });
  }

  // 3개 이상 마크 터치 후 [초점 고정 & 록온 스캔 시작] 실행
  Future<void> _startLockOnScan() async {
    if (_manualSeeds.length < 2 || _latestImage == null || _camController == null) return;

    // 1. AF Hunting 방지를 위한 하드웨어 초점 및 노출 강제 고정(Focus/Exposure Lock)
    await _camController!.setFocusMode(FocusMode.locked);
    await _camController!.setExposureMode(ExposureMode.locked);

    final yPlane = _latestImage!.planes[0];
    final ok = _bridge!.bootstrapSeeds(
      yPlane.bytes,
      _latestImage!.width,
      _latestImage!.height,
      yPlane.bytesPerRow,
      _manualSeeds.map((s) => s.imagePx.dx).toList(),
      _manualSeeds.map((s) => s.imagePx.dy).toList(),
      _manualSeeds.map((s) => s.zMeter).toList(),
    );

    if (ok) {
      setState(() {
        _isLockedOn = true;
        _isStreaming = true;
        _waveHistory.clear();
      });
    }
  }

  void _finishAndShowReport() {
    setState(() => _isStreaming = false);
    final res = _bridge!.solveFinalSurvey();
    if (res.isValid == 0) return;

    showModalBottomSheet(
      context: context,
      isScrollControlled: true,
      backgroundColor: const Color(0xFF101820),
      builder: (_) => _buildReportSheet(res),
    );
  }

  @override
  void dispose() {
    _imuSub?.cancel();
    _camController?.dispose();
    _bridge?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (_camController == null || !_camController!.value.isInitialized) {
      return const Scaffold(body: Center(child: CircularProgressIndicator()));
    }

    return Scaffold(
      body: SafeArea(
        child: Column(
          children: [
            _buildTopHUD(),
            Expanded(
              child: LayoutBuilder(
                builder: (ctx, box) => GestureDetector(
                  onTapUp: (d) => _onTapViewfinder(d, box),
                  child: Stack(
                    fit: StackFit.expand,
                    children: [
                      CameraPreview(_camController!),
                      CustomPaint(
                        painter: DraftAROverlayPainter(
                          isLockedOn: _isLockedOn,
                          seeds: _manualSeeds,
                          spinePx: _spineCurvePx,
                          marks: _trackedMarks,
                          waterlinePx: _waterlinePx,
                          instantDraftM: _instantDraftM,
                          imgWidth: _latestImage?.width.toDouble() ?? 1920.0,
                          imgHeight: _latestImage?.height.toDouble() ?? 1080.0,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
            _buildBottomControlPanel(),
          ],
        ),
      ),
    );
  }

  Widget _buildTopHUD() {
    final progress = (_collectedFrames / 1200.0).clamp(0.0, 1.0);
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
      color: Colors.black87,
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(_selectedStation, style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.cyanAccent)),
          Text(
            _isLockedOn
                ? '순간 흘수: ${_instantDraftM.toStringAsFixed(3)} m (${(progress * 100).toInt()}%)'
                : '부트스트랩 대기: 마크 밑단을 터치하세요 (${_manualSeeds.length}/3)',
            style: const TextStyle(fontSize: 13, color: Colors.white),
          ),
        ],
      ),
    );
  }

  Widget _buildBottomControlPanel() {
    return Container(
      padding: const EdgeInsets.all(14),
      color: const Color(0xFF121212),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          if (!_isLockedOn)
            Row(
              children: [
                const Text('다음 터치 마크 Z(m): '),
                IconButton(
                  icon: const Icon(Icons.remove_circle_outline),
                  onPressed: () => setState(() => _nextSeedDraftM = double.parse((_nextSeedDraftM - 0.2).toStringAsFixed(1))),
                ),
                Text('${_nextSeedDraftM.toStringAsFixed(1)} m',
                    style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: Colors.amberAccent)),
                IconButton(
                  icon: const Icon(Icons.add_circle_outline),
                  onPressed: () => setState(() => _nextSeedDraftM = double.parse((_nextSeedDraftM + 0.2).toStringAsFixed(1))),
                ),
                const Spacer(),
                TextButton(
                  onPressed: () => setState(() => _manualSeeds.clear()),
                  child: const Text('초기화'),
                ),
              ],
            ),
          const SizedBox(height: 8),
          Row(
            children: [
              Expanded(
                child: ElevatedButton.icon(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: _isLockedOn ? Colors.redAccent : Colors.teal,
                    padding: const EdgeInsets.symmetric(vertical: 14),
                  ),
                  icon: Icon(_isLockedOn ? Icons.stop_circle : Icons.center_focus_strong),
                  label: Text(_isLockedOn
                      ? '스캔 완료 및 AWLS 흘수 산출 (${(_collectedFrames / 30).toStringAsFixed(1)}s)'
                      : '초점 고정(AF Lock) & PCHIP 스파인 록온 시작'),
                  onPressed: _isLockedOn ? _finishAndShowReport : _startLockOnScan,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildReportSheet(FinalSurveyResult res) {
    final pass = res.reliabilityPass == 1;
    return Padding(
      padding: const EdgeInsets.all(24),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              const Text('AWLS + Sainflou 최종 흘수 검정서',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
              Chip(
                label: Text(pass ? 'PASS (신뢰도 적합)' : 'WARNING (추가 스캔 권장)'),
                backgroundColor: pass ? Colors.green.shade800 : Colors.orange.shade900,
              ),
            ],
          ),
          const Divider(height: 24),
          Text(
            '${res.trueDraftM.toStringAsFixed(3)} m',
            style: const TextStyle(fontSize: 44, fontWeight: FontWeight.bold, color: Colors.cyanAccent),
          ),
          const SizedBox(height: 12),
          Text('• 단순 산술 평균 (런업 오염): ${res.simpleMeanM.toStringAsFixed(3)} m'),
          Text('• AWLS 파저/제로크로싱 중심: ${res.apparentMeanM.toStringAsFixed(3)} m'),
          Text('• Sainflou 반사 수위 물리 차감: -${res.sainflouDeductMm.toStringAsFixed(1)} mm'),
          const Divider(height: 20),
          Text('• 관측 너울 파고(H) / 주기(Tp): ${res.swellHeightM.toStringAsFixed(2)} m / ${res.swellPeriodS.toStringAsFixed(1)} s'),
          Text('• 통과 너울 주기 / 파저 RMSE: ${res.observedCycles.toStringAsFixed(1)} 주기 / ±${res.troughRmseMm.toStringAsFixed(1)} mm'),
          Text('• 전반/후반 Split-Half 편차: ${res.splitHalfDiffMm.toStringAsFixed(1)} mm (기준 ≤ 12.0 mm)'),
          const SizedBox(height: 20),
        ],
      ),
    );
  }
}

// ============================================================================
// AR 뷰파인더 오버레이 페인터 (PCHIP 곡면 스파인, 0.2m 마크 트랙, 수면 접선)
// ============================================================================
class DraftAROverlayPainter extends CustomPainter {
  final bool isLockedOn;
  final List<SeedPoint> seeds;
  final List<Offset> spinePx;
  final List<MapEntry<double, Offset>> marks;
  final Offset waterlinePx;
  final double instantDraftM;
  final double imgWidth;
  final double imgHeight;

  DraftAROverlayPainter({
    required this.isLockedOn,
    required this.seeds,
    required this.spinePx,
    required this.marks,
    required this.waterlinePx,
    required this.instantDraftM,
    required this.imgWidth,
    required this.imgHeight,
  });

  Offset _toScreen(Offset pt, Size size) {
    return Offset((pt.dx / imgWidth) * size.width, (pt.dy / imgHeight) * size.height);
  }

  @override
  void paint(Canvas canvas, Size size) {
    if (!isLockedOn) {
      // 부트스트랩 터치 시드 표시
      final seedPaint = Paint()..color = Colors.amberAccent..style = PaintingStyle.stroke..strokeWidth = 2.5;
      for (final s in seeds) {
        canvas.drawCircle(s.screenPx, 10, seedPaint);
      }
      return;
    }

    // 1. PCHIP 마크 스파인 곡선 렌더링 (초록색 실선)
    if (spinePx.length >= 2) {
      final path = Path();
      final p0 = _toScreen(spinePx.first, size);
      path.moveTo(p0.dx, p0.dy);
      for (int i = 1; i < spinePx.length; i++) {
        final pi = _toScreen(spinePx[i], size);
        path.lineTo(pi.dx, pi.dy);
      }
      canvas.drawPath(
        path,
        Paint()..color = Colors.greenAccent..style = PaintingStyle.stroke..strokeWidth = 2.5,
      );
    }

    // 2. 자동 증식·록온된 0.2m 간격 마크 눈금 표시
    final markPaint = Paint()..color = Colors.cyanAccent..strokeWidth = 2.0;
    for (final m in marks) {
      final sc = _toScreen(m.value, size);
      canvas.drawLine(Offset(sc.dx - 14, sc.dy), Offset(sc.dx + 14, sc.dy), markPaint);
    }

    // 3. 실시간 서브픽셀 수면(Waterline) 교차선 표시 (적색 강조선)
    final wl = _toScreen(waterlinePx, size);
    canvas.drawLine(
      Offset(wl.dx - 55, wl.dy),
      Offset(wl.dx + 55, wl.dy),
      Paint()..color = Colors.redAccent..strokeWidth = 3.0,
    );
  }

  @override
  bool shouldRepaint(covariant DraftAROverlayPainter oldDelegate) => true;
}
