#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QTimer>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void connectToSerial();
    void readSerialData();
    void clearDisplay();
    void handleSerialError(QSerialPort::SerialPortError error);
    void onTimeWindowChanged();
    void onADCTimeWindowChanged();
    void togglePressureRecording();
    void toggleADCRecording();
    void populateSerialPorts();
    void populateFileList();
    void onSaveMetadataClicked();

private:
    Ui::MainWindow *ui;
    QSerialPort *serialPort;
    QTimer *connectionTimer;
    int messageCount;
    QByteArray dataBuffer; // Buffer to accumulate incoming data

    // Recording related members
    bool isPressureRecording;
    bool isADCRecording;
    QVector<QPair<qint64, double>> recordedData; // Stores time and pressure pairs
    QVector<QPair<qint64, int>> recordedADCData; // Stores time and ADC pairs
    qint64 pressureRecordingStartTime;
    qint64 adcRecordingStartTime;

    // Auto-cycle recording members
    bool isCycleRecordingActive;
    QVector<QPair<qint64, int>> recordedCycleData;
    qint64 cycleRecordingStartTime;

    // Chart components
    QChart *chart;
    QLineSeries *pressureSeries;
    QValueAxis *axisX;
    QValueAxis *axisY;
    qint64 startTime;
    double maxDataPoints;
    int pressureTimeWindowSeconds; // Time window for pressure chart
    int adcTimeWindowSeconds;      // Time window for ADC chart

    // ADC Chart components
    QChart *adcChart;
    QLineSeries *adcSeries;
    QValueAxis *adcAxisX;
    QValueAxis *adcAxisY;

    void updateConnectionStatus(bool connected);
    void appendSerialData(const QString &data);
    void processCompleteLines();
    void setupPressureChart();
    void setupADCChart();
    void updatePressureChart(double pressure, qint64 timestamp);
    void updateADCChart(int adcValue, qint64 timestamp);
    void updatePressureTimeWindow();
    void updateADCTimeWindow();
    double parsePresssureValue(const QString &line);
    int parseADCValue(const QString &line);
    void savePressureData();
    void saveADCData();
    void saveCycleData(); // New function for auto-cycle recording
    void updateMetadataFile(const QString &fileName, const QString &directory, double durationSeconds);
    void createConfigYamlIfNotExists(const QString &directoryPath);
    void sendCommand(const QByteArray &command);
};
#endif // MAINWINDOW_H
