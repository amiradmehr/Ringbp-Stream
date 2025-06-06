#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDateTime>
#include <QScrollBar>
#include <QMessageBox>
#include <QRegularExpression>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QSerialPortInfo>
#include <QFileDialog>
#include <QTimer>
#include <QVariantAnimation>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), serialPort(nullptr), connectionTimer(new QTimer(this)), messageCount(0), startTime(0), maxDataPoints(100) // Keep last 100 data points
      ,
      pressureTimeWindowSeconds(5), // Default to 5 seconds for pressure
      adcTimeWindowSeconds(5)       // Default to 5 seconds for ADC
{
    ui->setupUi(this);

    // Set initial sizes for the splitters
    // Main splitter (horizontal): controlPanel vs displaySplitter
    // Give less space to controlPanel (e.g., 1 part) and more to displaySplitter (e.g., 4 parts)
    ui->mainSplitter->setSizes(QList<int>() << 150 << 850);

    // Display splitter (vertical): chartsGroupBox vs serialDataGroupBox
    // Give more space to chartsGroupBox (e.g., 7 parts) and less to serialDataGroupBox (e.g., 1 part)
    ui->displaySplitter->setSizes(QList<int>() << 700 << 100);

    // Ensure light theme for the main window and key elements
    this->setStyleSheet(
        "QMainWindow { background-color: white; }"
        "QTextEdit { "
        "    background-color: white; "
        "    color: black; "
        "    border: 1px solid #ccc; "
        "    border-radius: 4px; "
        "}"
        "QLabel { color: black; }"
        "QPushButton { "
        "    background-color: #f0f0f0; "
        "    color: black; "
        "    border: 1px solid #ccc; "
        "    padding: 5px 10px; "
        "    border-radius: 4px; "
        "}"
        "QPushButton:hover { background-color: #e0e0e0; }"
        "QPushButton:pressed { background-color: #d0d0d0; }"
        "QGroupBox { "
        "    font-weight: bold; "
        "    border: 2px solid #ccc; "
        "    border-radius: 5px; "
        "    margin: 5px 0px; "
        "    padding-top: 10px; "
        "}"
        "QGroupBox::title { "
        "    subcontrol-origin: margin; "
        "    left: 10px; "
        "    padding: 0 5px 0 5px; "
        "}"
        "QComboBox { "
        "    background-color: white; "
        "    color: black; "
        "    border: 1px solid #ccc; "
        "    padding: 3px 5px; "
        "    border-radius: 4px; "
        "}"
        "QComboBox:hover { border-color: #3498db; }");

    // Initialize serial port
    serialPort = new QSerialPort(this);

    // Setup chart
    setupPressureChart();
    setupADCChart();

    // Set default time window selection (5 sec = index 1)
    ui->timeWindowComboBox->setCurrentIndex(1);
    ui->adcTimeWindowComboBox->setCurrentIndex(1);

    // Initialize recording state
    isPressureRecording = false;
    isADCRecording = false;
    recordedData.clear();
    recordedADCData.clear();
    pressureRecordingStartTime = 0;
    adcRecordingStartTime = 0;

    // Initialize auto-cycle recording state
    isCycleRecordingActive = false;
    recordedCycleData.clear();
    cycleRecordingStartTime = 0;

    // Populate serial ports
    populateSerialPorts();

    // Populate file list for the default directory
    populateFileList();

    // Connect button signals
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::connectToSerial);
    connect(ui->clearButton, &QPushButton::clicked, this, &MainWindow::clearDisplay);
    connect(ui->refreshPortsButton, &QPushButton::clicked, this, &MainWindow::populateSerialPorts);
    connect(ui->recordButton, &QPushButton::clicked, this, &MainWindow::togglePressureRecording);
    connect(ui->adcRecordButton, &QPushButton::clicked, this, &MainWindow::toggleADCRecording);
    connect(ui->refreshFilesButton, &QPushButton::clicked, this, &MainWindow::populateFileList);
    connect(ui->saveMetadataButton, &QPushButton::clicked, this, &MainWindow::onSaveMetadataClicked);
    connect(ui->recordingPathLineEdit, &QLineEdit::editingFinished, this, &MainWindow::populateFileList);

    // Connect motor control buttons
    connect(ui->forwardButton, &QPushButton::clicked, [this]()
            { sendCommand("]"); });
    connect(ui->backwardButton, &QPushButton::clicked, [this]()
            { sendCommand("["); });
    connect(ui->cuffCycleButton, &QPushButton::clicked, [this]()
            { sendCommand("j"); });
    connect(ui->releaseButton, &QPushButton::clicked, [this]()
            { sendCommand("k"); });
    connect(ui->stopButton, &QPushButton::clicked, [this]()
            { sendCommand("x"); });
    connect(ui->calibrateButton, &QPushButton::clicked, [this]()
            { sendCommand("c"); });
    connect(ui->displayStallButton, &QPushButton::clicked, [this]()
            { sendCommand("d"); });

    // Connect time window changes
    connect(ui->timeWindowComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onTimeWindowChanged);
    connect(ui->adcTimeWindowComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onADCTimeWindowChanged);

    // Setup connection timer for auto-reconnect attempts
    connect(connectionTimer, &QTimer::timeout, this, &MainWindow::connectToSerial);

    // Initial status (this will call updateConnectionStatus, disabling buttons)
    updateConnectionStatus(false);

    // Welcome message
    appendSerialData("=== Arduino Pressure Monitor Started ===");
    appendSerialData("Waiting for connection to COM10...");
}

MainWindow::~MainWindow()
{
    if (serialPort && serialPort->isOpen())
    {
        serialPort->close();
    }
    delete ui;
}

void MainWindow::setupPressureChart()
{
    // Create chart and series
    chart = new QChart();
    pressureSeries = new QLineSeries();

    // Configure series
    pressureSeries->setName("Pressure (mmHg)");
    pressureSeries->setColor(QColor(52, 152, 219)); // Nice blue color

    // Add series to chart
    chart->addSeries(pressureSeries);
    chart->setTitle("Real-time Pressure Monitoring (mmHg)");
    chart->setAnimationOptions(QChart::NoAnimation); // Disable for real-time

    // Create axes
    axisX = new QValueAxis();
    axisY = new QValueAxis();

    // Configure X axis (time)
    axisX->setTitleText("Time (seconds)");
    axisX->setRange(0, pressureTimeWindowSeconds);      // Use pressure time window
    axisX->setTickCount(pressureTimeWindowSeconds + 1); // One tick per second

    // Configure Y axis (pressure in mmHg)
    axisY->setTitleText("Pressure (mmHg)");
    axisY->setRange(-50, 50); // Start with a range that includes negative values
    axisY->setTickCount(6);

    // Attach axes
    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);
    pressureSeries->attachAxis(axisX);
    pressureSeries->attachAxis(axisY);

    // Set chart to the view
    ui->pressureChart->setChart(chart);
    ui->pressureChart->setRenderHint(QPainter::Antialiasing);
}

void MainWindow::setupADCChart()
{
    // Create chart and series
    adcChart = new QChart();
    adcSeries = new QLineSeries();

    // Configure series
    adcSeries->setName("ADC Value");
    adcSeries->setColor(QColor(231, 76, 60)); // Nice red color

    // Add series to chart
    adcChart->addSeries(adcSeries);
    adcChart->setTitle("Real-time ADC Monitoring (nV)");
    adcChart->setAnimationOptions(QChart::NoAnimation); // Disable for real-time

    // Create axes
    adcAxisX = new QValueAxis();
    adcAxisY = new QValueAxis();

    // Configure X axis (time)
    adcAxisX->setTitleText("Time (seconds)");
    adcAxisX->setRange(0, adcTimeWindowSeconds);
    adcAxisX->setTickCount(adcTimeWindowSeconds + 1);

    // Configure Y axis (ADC value)
    adcAxisY->setTitleText("ADC Value (nV)");
    adcAxisY->setRange(-100, 100); // Initial range that includes negative values
    adcAxisY->setTickCount(6);

    // Attach axes
    adcChart->addAxis(adcAxisX, Qt::AlignBottom);
    adcChart->addAxis(adcAxisY, Qt::AlignLeft);
    adcSeries->attachAxis(adcAxisX);
    adcSeries->attachAxis(adcAxisY);

    // Set chart to the view
    ui->adcChart->setChart(adcChart);
    ui->adcChart->setRenderHint(QPainter::Antialiasing);
}

void MainWindow::connectToSerial()
{
    if (serialPort->isOpen())
    {
        // Disconnect
        serialPort->close();
        updateConnectionStatus(false);
        return;
    }

    QString selectedPort = ui->portComboBox->currentText();
    if (selectedPort.isEmpty())
    {
        appendSerialData("Error: No serial port selected.");
        return;
    }

    // Setup serial port
    serialPort->setPortName(selectedPort);
    serialPort->setBaudRate(QSerialPort::Baud115200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    // Try to open the port
    if (serialPort->open(QIODevice::ReadWrite))
    {
        updateConnectionStatus(true);
        connectionTimer->stop();
        dataBuffer.clear(); // Clear buffer for fresh start

        // Connect data ready signal
        connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::readSerialData);
        connect(serialPort, QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
                this, &MainWindow::handleSerialError);

        appendSerialData(QString("=== Connected to %1 ===").arg(selectedPort));
        appendSerialData("Listening for Arduino data...");
    }
    else
    {
        updateConnectionStatus(false);
        appendSerialData(QString("Failed to connect to %1: %2").arg(selectedPort, serialPort->errorString()));

        // Start auto-reconnect timer (try every 3 seconds)
        if (!connectionTimer->isActive())
        {
            connectionTimer->start(3000);
            appendSerialData("Auto-reconnect enabled (trying every 3 seconds)...");
        }
    }
}

void MainWindow::readSerialData()
{
    if (!serialPort)
        return;

    // Read all available data and append to buffer
    QByteArray newData = serialPort->readAll();
    if (!newData.isEmpty())
    {
        dataBuffer.append(newData);
        processCompleteLines();
    }
}

void MainWindow::processCompleteLines()
{
    // Process all complete lines in the buffer
    while (dataBuffer.contains('\n'))
    {
        int lineEndIndex = dataBuffer.indexOf('\n');
        QByteArray lineData = dataBuffer.left(lineEndIndex);
        dataBuffer.remove(0, lineEndIndex + 1);

        QString line = QString::fromUtf8(lineData).trimmed();

        if (line.isEmpty())
        {
            continue;
        }

        // --- Handle Auto-Cycle Recording Triggers ---
        // The start trigger is the message that a stall has been detected.
        if (line.contains("STALL DETECTED") && !isCycleRecordingActive)
        {
            isCycleRecordingActive = true;
            recordedCycleData.clear();
            cycleRecordingStartTime = 0;
            appendSerialData("=== Auto-cycle recording STARTED (Stall Detected) ===");
        }
        // The stop trigger is the message that the final movement is reported,
        // which happens right before the status goes back to IDLE.
        else if (line.contains("POSITION_MOVEMENT") && isCycleRecordingActive)
        {
            isCycleRecordingActive = false;
            appendSerialData("=== Auto-cycle recording STOPPED (Cycle End) ===");
            if (!recordedCycleData.isEmpty())
            {
                saveCycleData();
            }
        }

        // --- Continue with normal processing ---
        QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
        QString message = QString("[%1] %2").arg(timestamp, line);

        appendSerialData(message);
        messageCount++;
        ui->dataCountLabel->setText(QString("Messages: %1").arg(messageCount));

        // Parse and plot pressure value
        double pressure = parsePresssureValue(line);
        int adcValue = parseADCValue(line);

        if (pressure != -1 || adcValue != -1)
        {
            qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
            if (startTime == 0)
            {
                startTime = currentTime;
            }

            if (pressure != -1)
            {
                updatePressureChart(pressure, currentTime);
                ui->currentPressureLabel->setText(QString("Current Pressure: %1 mmHg").arg(pressure, 0, 'f', 2));
            }

            if (adcValue != -1)
            {
                updateADCChart(adcValue, currentTime);
            }

            // Handle recording for all modes
            QRegularExpression elapsedRegex(R"(Elapsed:\s*(\d+)ms)");
            QRegularExpressionMatch match = elapsedRegex.match(line);
            if (match.hasMatch())
            {
                qint64 elapsedTime = match.captured(1).toLongLong();

                // Manual Pressure Recording
                if (isPressureRecording && pressure != -1)
                {
                    if (pressureRecordingStartTime == 0)
                    {
                        pressureRecordingStartTime = elapsedTime;
                    }
                    recordedData.append(qMakePair(elapsedTime - pressureRecordingStartTime, pressure));
                }

                // Manual ADC Recording
                if (isADCRecording && adcValue != -1)
                {
                    if (adcRecordingStartTime == 0)
                    {
                        adcRecordingStartTime = elapsedTime;
                    }
                    recordedADCData.append(qMakePair(elapsedTime - adcRecordingStartTime, adcValue));
                }

                // Auto-cycle ADC Recording
                if (isCycleRecordingActive && adcValue != -1)
                {
                    if (cycleRecordingStartTime == 0)
                    {
                        cycleRecordingStartTime = elapsedTime;
                    }
                    recordedCycleData.append(qMakePair(elapsedTime - cycleRecordingStartTime, adcValue));
                }
            }
        }
    }
}

double MainWindow::parsePresssureValue(const QString &line)
{
    // Parse pressure from line like: "ADC: 2346 | Press: 4.59 | Elapsed: 49402ms | Status: IDLE"
    // Allows for optional negative sign
    QRegularExpression pressureRegex(R"(Press:\s*(-?[0-9]+\.?[0-9]*))");
    QRegularExpressionMatch match = pressureRegex.match(line);

    if (match.hasMatch())
    {
        bool ok;
        double pressure = match.captured(1).toDouble(&ok);
        if (ok)
        {
            return pressure - 3.5;
        }
    }
    return -1; // Invalid pressure
}

int MainWindow::parseADCValue(const QString &line)
{
    // Parse ADC from line like: "ADC: 2346 | Press: 4.59 | Elapsed: 49402ms | Status: IDLE"
    // Allows for optional negative sign
    QRegularExpression adcRegex(R"(ADC:\s*(-?\d+))");
    QRegularExpressionMatch match = adcRegex.match(line);

    if (match.hasMatch())
    {
        bool ok;
        int adcValue = match.captured(1).toInt(&ok);
        if (ok)
        {
            return adcValue;
        }
    }
    return -1; // Invalid ADC value
}

void MainWindow::updatePressureChart(double pressure, qint64 timestamp)
{
    // Calculate time in seconds from start
    double timeInSeconds = (timestamp - startTime) / 1000.0;

    // Add new data point
    pressureSeries->append(timeInSeconds, pressure);

    // Remove old data points that are outside the maximum time window
    double maxKeepTime = 30.0;
    while (pressureSeries->count() > 1)
    {
        QList<QPointF> points = pressureSeries->points();
        if (points.first().x() < timeInSeconds - maxKeepTime)
        {
            pressureSeries->remove(0);
        }
        else
        {
            break;
        }
    }

    // Limit by count
    while (pressureSeries->count() > 1000)
    {
        pressureSeries->remove(0);
    }

    // Auto-adjust X axis
    QList<QPointF> points = pressureSeries->points();
    if (!points.isEmpty())
    {
        double latestTime = points.last().x();
        double windowStartTime = latestTime - pressureTimeWindowSeconds;
        double displayStartTime = qMax(0.0, windowStartTime);
        double displayEndTime = latestTime + 0.1;

        axisX->setRange(displayStartTime, displayEndTime);
    }

    // Auto-adjust Y axis based on visible data points
    if (pressureSeries->count() > 2)
    {
        QList<QPointF> points = pressureSeries->points();
        double latestTime = points.last().x();
        double windowStartTime = latestTime - pressureTimeWindowSeconds;

        double minY = points.last().y();
        double maxY = points.last().y();
        bool hasVisibleData = false;

        for (const QPointF &point : points)
        {
            if (point.x() >= windowStartTime)
            {
                minY = qMin(minY, point.y());
                maxY = qMax(maxY, point.y());
                hasVisibleData = true;
            }
        }

        if (hasVisibleData)
        {
            double margin = (maxY - minY) * 0.1;
            if (margin < 5)
                margin = 5; // Minimum margin for mmHg

            axisY->setRange(minY - margin, maxY + margin);
        }
    }
}

void MainWindow::updateADCChart(int adcValue, qint64 timestamp)
{
    // Calculate time in seconds from start
    double timeInSeconds = (timestamp - startTime) / 1000.0;

    // Add new data point
    adcSeries->append(timeInSeconds, adcValue);

    // Remove old data points that are outside the maximum time window
    double maxKeepTime = 30.0;
    while (adcSeries->count() > 1)
    {
        QList<QPointF> points = adcSeries->points();
        if (points.first().x() < timeInSeconds - maxKeepTime)
        {
            adcSeries->remove(0);
        }
        else
        {
            break;
        }
    }

    // Limit by count
    while (adcSeries->count() > 1000)
    {
        adcSeries->remove(0);
    }

    // Auto-adjust X axis
    QList<QPointF> points = adcSeries->points();
    if (!points.isEmpty())
    {
        double latestTime = points.last().x();
        double windowStartTime = latestTime - adcTimeWindowSeconds;
        double displayStartTime = qMax(0.0, windowStartTime);
        double displayEndTime = latestTime + 0.1;

        adcAxisX->setRange(displayStartTime, displayEndTime);
    }

    // Auto-adjust Y axis based on visible data points
    if (adcSeries->count() > 2)
    {
        QList<QPointF> points = adcSeries->points();
        double latestTime = points.last().x();
        double windowStartTime = latestTime - adcTimeWindowSeconds;

        double minY = points.last().y();
        double maxY = points.last().y();
        bool hasVisibleData = false;

        for (const QPointF &point : points)
        {
            if (point.x() >= windowStartTime)
            {
                minY = qMin(minY, point.y());
                maxY = qMax(maxY, point.y());
                hasVisibleData = true;
            }
        }

        if (hasVisibleData)
        {
            double margin = (maxY - minY) * 0.1;
            if (margin < 100)
                margin = 100; // Minimum margin for ADC values

            adcAxisY->setRange(minY - margin, maxY + margin);
        }
    }
}

void MainWindow::clearDisplay()
{
    ui->serialDataDisplay->clear();
    messageCount = 0;
    ui->dataCountLabel->setText("Messages: 0");
    ui->currentPressureLabel->setText("Current Pressure: -- mmHg");

    // Clear chart data
    pressureSeries->clear();
    adcSeries->clear();
    startTime = 0;
    axisX->setRange(0, pressureTimeWindowSeconds);
    adcAxisX->setRange(0, adcTimeWindowSeconds);
    axisY->setRange(-50, 50);
    adcAxisY->setRange(-100, 100); // Reset to initial range that includes negative values

    // Reset recording states
    isPressureRecording = false;
    isADCRecording = false;
    ui->recordButton->setText("Start Recording");
    ui->recordButton->setStyleSheet("");
    ui->adcRecordButton->setText("Start Recording");
    ui->adcRecordButton->setStyleSheet("");

    // Reset auto-cycle recording state
    isCycleRecordingActive = false;
    recordedCycleData.clear();

    appendSerialData("=== Display and Charts Cleared ===");
}

void MainWindow::handleSerialError(QSerialPort::SerialPortError error)
{
    if (error != QSerialPort::NoError)
    {
        appendSerialData(QString("Serial Error: %1").arg(serialPort->errorString()));

        if (serialPort->isOpen())
        {
            serialPort->close();
            updateConnectionStatus(false);
            ui->connectButton->setText("Connect");

            // Start auto-reconnect
            if (!connectionTimer->isActive())
            {
                connectionTimer->start(3000);
                appendSerialData("Attempting to reconnect...");
            }

            // Stop and clear any active cycle recording
            if (isCycleRecordingActive)
            {
                isCycleRecordingActive = false;
                recordedCycleData.clear();
                appendSerialData("=== Auto-cycle recording cancelled due to disconnect ===");
            }
        }
    }
}

void MainWindow::updateConnectionStatus(bool connected)
{
    if (connected)
    {
        ui->connectButton->setText(QString("Disconnect from %1").arg(serialPort->portName()));
        ui->connectButton->setStyleSheet("background-color: #27ae60; color: white;"); // Green
        ui->portComboBox->setEnabled(false);
        ui->refreshPortsButton->setEnabled(false);
        ui->recordButton->setEnabled(true);
        ui->adcRecordButton->setEnabled(true);
        // Reset stylesheet if it was greyed out, but respect recording state style
        if (!isPressureRecording)
            ui->recordButton->setStyleSheet("");
        else
            ui->recordButton->setStyleSheet("background-color: #e74c3c; color: white;");
        if (!isADCRecording)
            ui->adcRecordButton->setStyleSheet("");
        else
            ui->adcRecordButton->setStyleSheet("background-color: #e74c3c; color: white;");

        // Enable motor control buttons
        ui->forwardButton->setEnabled(true);
        ui->backwardButton->setEnabled(true);
        ui->cuffCycleButton->setEnabled(true);
        ui->releaseButton->setEnabled(true);
        ui->stopButton->setEnabled(true);
        ui->calibrateButton->setEnabled(true);
        ui->displayStallButton->setEnabled(true);

        // Reset motor control button styles
        ui->forwardButton->setStyleSheet("");
        ui->backwardButton->setStyleSheet("");
        ui->cuffCycleButton->setStyleSheet("");
        ui->releaseButton->setStyleSheet("");
        ui->stopButton->setStyleSheet("");
        ui->calibrateButton->setStyleSheet("");
        ui->displayStallButton->setStyleSheet("");
    }
    else
    {
        ui->connectButton->setText("Connect");
        ui->connectButton->setStyleSheet("background-color: #e74c3c; color: white;"); // Red
        ui->portComboBox->setEnabled(true);
        ui->refreshPortsButton->setEnabled(true);
        ui->recordButton->setEnabled(false);
        ui->adcRecordButton->setEnabled(false);
        ui->recordButton->setStyleSheet("background-color: #d3d3d3; color: #a0a0a0;");    // Greyed out
        ui->adcRecordButton->setStyleSheet("background-color: #d3d3d3; color: #a0a0a0;"); // Greyed out

        // If recording was active, stop it
        if (isPressureRecording)
        {
            togglePressureRecording(); // This will also update button text/style if it were enabled
        }
        if (isADCRecording)
        {
            toggleADCRecording(); // This will also update button text/style if it were enabled
        }

        // Stop and clear any active cycle recording
        if (isCycleRecordingActive)
        {
            isCycleRecordingActive = false;
            recordedCycleData.clear();
            appendSerialData("=== Auto-cycle recording cancelled due to disconnect ===");
        }

        // Disable motor control buttons
        const QString disabledStyle = "background-color: #d3d3d3; color: #a0a0a0;";
        ui->forwardButton->setEnabled(false);
        ui->forwardButton->setStyleSheet(disabledStyle);
        ui->backwardButton->setEnabled(false);
        ui->backwardButton->setStyleSheet(disabledStyle);
        ui->cuffCycleButton->setEnabled(false);
        ui->cuffCycleButton->setStyleSheet(disabledStyle);
        ui->releaseButton->setEnabled(false);
        ui->releaseButton->setStyleSheet(disabledStyle);
        ui->stopButton->setEnabled(false);
        ui->stopButton->setStyleSheet(disabledStyle);
        ui->calibrateButton->setEnabled(false);
        ui->calibrateButton->setStyleSheet(disabledStyle);
        ui->displayStallButton->setEnabled(false);
        ui->displayStallButton->setStyleSheet(disabledStyle);
    }
}

void MainWindow::appendSerialData(const QString &data)
{
    ui->serialDataDisplay->append(data);

    // Auto-scroll to bottom
    QScrollBar *scrollBar = ui->serialDataDisplay->verticalScrollBar();
    scrollBar->setValue(scrollBar->maximum());
}

void MainWindow::onTimeWindowChanged()
{
    // Get the current selection and map to time window in seconds
    int index = ui->timeWindowComboBox->currentIndex();
    int newTimeWindowSeconds;

    switch (index)
    {
    case 0:
        newTimeWindowSeconds = 1;
        break;
    case 1:
        newTimeWindowSeconds = 5;
        break;
    case 2:
        newTimeWindowSeconds = 10;
        break;
    default:
        newTimeWindowSeconds = 5;
        break;
    }

    if (newTimeWindowSeconds != pressureTimeWindowSeconds)
    {
        pressureTimeWindowSeconds = newTimeWindowSeconds;
        appendSerialData(QString("=== Pressure Time Window Changed to %1 seconds ===").arg(pressureTimeWindowSeconds));
        updatePressureTimeWindow();
    }
}

void MainWindow::updatePressureTimeWindow()
{
    if (!axisX || !pressureSeries)
    {
        appendSerialData("Warning: Pressure Chart not ready for time window update");
        return;
    }

    if (pressureSeries->count() > 0)
    {
        QList<QPointF> points = pressureSeries->points();
        if (!points.isEmpty())
        {
            double latestTime = points.last().x();
            double windowStartTime = latestTime - pressureTimeWindowSeconds;
            double startTime = qMax(0.0, windowStartTime);
            double endTime = qMax(latestTime + 0.1, startTime + pressureTimeWindowSeconds);

            axisX->setRange(startTime, endTime);
        }
    }
    else
    {
        axisX->setRange(0, pressureTimeWindowSeconds);
    }

    axisX->setTickCount(qMin(pressureTimeWindowSeconds + 1, 11));
}

void MainWindow::onADCTimeWindowChanged()
{
    // Get the current selection and map to time window in seconds
    int index = ui->adcTimeWindowComboBox->currentIndex();
    int newTimeWindowSeconds;

    switch (index)
    {
    case 0:
        newTimeWindowSeconds = 1;
        break;
    case 1:
        newTimeWindowSeconds = 5;
        break;
    case 2:
        newTimeWindowSeconds = 10;
        break;
    default:
        newTimeWindowSeconds = 5;
        break;
    }

    if (newTimeWindowSeconds != adcTimeWindowSeconds)
    {
        adcTimeWindowSeconds = newTimeWindowSeconds;
        appendSerialData(QString("=== ADC Time Window Changed to %1 seconds ===").arg(adcTimeWindowSeconds));
        updateADCTimeWindow();
    }
}

void MainWindow::updateADCTimeWindow()
{
    if (!adcAxisX || !adcSeries)
    {
        appendSerialData("Warning: ADC Chart not ready for time window update");
        return;
    }

    if (adcSeries->count() > 0)
    {
        QList<QPointF> points = adcSeries->points();
        if (!points.isEmpty())
        {
            double latestTime = points.last().x();
            double windowStartTime = latestTime - adcTimeWindowSeconds;
            double startTime = qMax(0.0, windowStartTime);
            double endTime = qMax(latestTime + 0.1, startTime + adcTimeWindowSeconds);

            adcAxisX->setRange(startTime, endTime);
        }
    }
    else
    {
        adcAxisX->setRange(0, adcTimeWindowSeconds);
    }

    adcAxisX->setTickCount(qMin(adcTimeWindowSeconds + 1, 11));
}

void MainWindow::togglePressureRecording()
{
    if (!isPressureRecording)
    {
        // Start recording
        isPressureRecording = true;
        recordedData.clear();
        pressureRecordingStartTime = 0;
        ui->recordButton->setText("Stop Recording");
        ui->recordButton->setStyleSheet("background-color: #e74c3c; color: white;");
        appendSerialData("=== Started Recording Pressure Data ===");
    }
    else
    {
        // Stop recording and save data
        isPressureRecording = false;
        ui->recordButton->setText("Start Recording");
        ui->recordButton->setStyleSheet("");

        if (!recordedData.isEmpty())
        {
            savePressureData();
        }

        appendSerialData("=== Stopped Recording Pressure Data ===");
    }
}

void MainWindow::toggleADCRecording()
{
    if (!isADCRecording)
    {
        // Start recording
        isADCRecording = true;
        recordedADCData.clear();
        adcRecordingStartTime = 0;
        ui->adcRecordButton->setText("Stop Recording");
        ui->adcRecordButton->setStyleSheet("background-color: #e74c3c; color: white;");
        appendSerialData("=== Started Recording ADC Data ===");
    }
    else
    {
        // Stop recording and save data
        isADCRecording = false;
        ui->adcRecordButton->setText("Start Recording");
        ui->adcRecordButton->setStyleSheet("");

        if (!recordedADCData.isEmpty())
        {
            saveADCData();
        }

        appendSerialData("=== Stopped Recording ADC Data ===");
    }
}

void MainWindow::savePressureData()
{
    QString dirPath = ui->recordingPathLineEdit->text().trimmed();
    if (dirPath.isEmpty())
    {
        dirPath = "."; // Default to current directory if empty
    }

    QDir dir(dirPath);
    if (!dir.exists())
    {
        if (!dir.mkpath(".")) // mkpath creates parent directories if needed
        {
            appendSerialData(QString("Error: Could not create directory %1").arg(dirPath));
            return;
        }
        appendSerialData(QString("Created directory: %1").arg(dirPath));
    }

    createConfigYamlIfNotExists(dirPath); // Create config.yaml if it doesn't exist

    // Generate filename with current date and time
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString pressureFilename = QString("%1/P_raw_%2.csv").arg(dirPath, timestamp);

    // Save pressure data
    QFile pressureFile(pressureFilename);
    if (pressureFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&pressureFile);
        out << "Time (ms),Pressure (mmHg)\n";
        for (const auto &data : recordedData)
        {
            out << data.first << "," << data.second << "\n";
        }
        pressureFile.close();
        appendSerialData(QString("=== Saved pressure data to %1 ===").arg(pressureFilename));

        // Update metadata
        double durationSeconds = 0;
        if (recordedData.size() > 1)
        {
            durationSeconds = (recordedData.last().first - recordedData.first().first) / 1000.0;
        }
        updateMetadataFile(QFileInfo(pressureFile).baseName(), dirPath, durationSeconds);
        populateFileList(); // Refresh file list
    }
    else
    {
        appendSerialData(QString("Error: Could not save pressure data to %1. Error: %2").arg(pressureFilename, pressureFile.errorString()));
    }
}

void MainWindow::saveADCData()
{
    QString dirPath = ui->recordingPathLineEdit->text().trimmed();
    if (dirPath.isEmpty())
    {
        dirPath = "."; // Default to current directory if empty
    }

    QDir dir(dirPath);
    if (!dir.exists())
    {
        if (!dir.mkpath(".")) // mkpath creates parent directories if needed
        {
            appendSerialData(QString("Error: Could not create directory %1").arg(dirPath));
            return;
        }
        appendSerialData(QString("Created directory: %1").arg(dirPath));
    }

    createConfigYamlIfNotExists(dirPath); // Create config.yaml if it doesn't exist

    // Generate filename with current date and time
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString adcFilename = QString("%1/ADC_raw_%2.csv").arg(dirPath, timestamp);

    // Save ADC data
    QFile adcFile(adcFilename);
    if (adcFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&adcFile);
        out << "Time (ms),ADC Value\n";
        for (const auto &data : recordedADCData)
        {
            out << data.first << "," << data.second << "\n";
        }
        adcFile.close();
        appendSerialData(QString("=== Saved ADC data to %1 ===").arg(adcFilename));

        // Update metadata
        double durationSeconds = 0;
        if (recordedADCData.size() > 1)
        {
            durationSeconds = (recordedADCData.last().first - recordedADCData.first().first) / 1000.0;
        }
        updateMetadataFile(QFileInfo(adcFile).baseName(), dirPath, durationSeconds);
        populateFileList(); // Refresh file list
    }
    else
    {
        appendSerialData(QString("Error: Could not save ADC data to %1. Error: %2").arg(adcFilename, adcFile.errorString()));
    }
}

void MainWindow::saveCycleData()
{
    QString dirPath = ui->recordingPathLineEdit->text().trimmed();
    if (dirPath.isEmpty())
    {
        dirPath = "."; // Default to current directory if empty
    }

    QDir dir(dirPath);
    if (!dir.exists())
    {
        if (!dir.mkpath(".")) // mkpath creates parent directories if needed
        {
            appendSerialData(QString("Error: Could not create directory %1").arg(dirPath));
            return;
        }
        appendSerialData(QString("Created directory: %1").arg(dirPath));
    }

    createConfigYamlIfNotExists(dirPath); // Create config.yaml if it doesn't exist

    // Generate filename with current date and time
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString cycleFilename = QString("%1/S_raw_Cycle_%2.csv").arg(dirPath, timestamp);

    // Save cycle data
    QFile cycleFile(cycleFilename);
    if (cycleFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&cycleFile);
        out << "Time (ms),ADC Value\n";
        for (const auto &data : recordedCycleData)
        {
            out << data.first << "," << data.second << "\n";
        }
        cycleFile.close();
        appendSerialData(QString("=== Saved auto-cycle data to %1 ===").arg(cycleFilename));

        // Update metadata
        double durationSeconds = 0;
        if (recordedCycleData.size() > 1)
        {
            durationSeconds = (recordedCycleData.last().first - recordedCycleData.first().first) / 1000.0;
        }
        updateMetadataFile(QFileInfo(cycleFile).baseName(), dirPath, durationSeconds);
        populateFileList(); // Refresh file list
    }
    else
    {
        appendSerialData(QString("Error: Could not save auto-cycle data to %1. Error: %2").arg(cycleFilename, cycleFile.errorString()));
    }
}

void MainWindow::createConfigYamlIfNotExists(const QString &directoryPath)
{
    QString configFilePath = directoryPath + "/config.yaml";

    if (QFile::exists(configFilePath))
    {
        return; // Config file already exists, do not overwrite.
    }

    QFile configFile(configFilePath);
    if (configFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&configFile);
        QString dirName = QDir(directoryPath).dirName();

        // Prepare a comprehensive YAML content describing all file types
        QString yamlContent = QString(
                                  R"END(
# Dataset Configuration

# Data directory settings
data_directory: "%1"
data_format: "csv"

# --- File Patterns ---
# The {date} and {time} placeholders correspond to the formats below.

# Pattern for automatically captured cuff cycle data after a stall
file_pattern:
  raw_data: "S_raw_Cycle_{date}_{time}.csv"
  date_format: "%Y%m%d"
  time_format: "%H%M%S"

# Pattern for manually recorded pressure data
pressure_data:
  file_pattern: "P_raw_{date}_{time}.csv"
  date_format: "yyyyMMdd"
  time_format: "HHmmss"

# Pattern for manually recorded raw ADC data
adc_data:
  file_pattern: "ADC_raw_{date}_{time}.csv"
  date_format: "yyyyMMdd"
  time_format: "HHmmss"

# --- Common Metadata ---
metadata_file: "signal_metadata.csv"

# --- Data Specifications ---
data_specs:
  sampling_rate: 100      # Hz (assumed, confirm from Arduino code)
  time_column: "Time (ms)"
  pressure_column: "Pressure (mmHg)"
  adc_column: "ADC Value"

# --- Processing Parameters ---
processing:
  normalization: true
  filtering:
    enabled: true
    type: "lowpass"
    cutoff_frequency: 20  # Hz

# --- Output Settings ---
output:
  save_processed: true
  processed_dir: "processed"
  file_format: "csv"

# --- Visualization Settings ---
visualization:
  plot_raw_data: true
  plot_processed_data: true
  save_plots: true
  plots_directory: "plots"
)END")
                                  .arg(dirName);

        out << yamlContent.trimmed(); // Use trimmed to remove any leading/trailing whitespace
        configFile.close();
        appendSerialData(QString("Created config.yaml in %1").arg(directoryPath));
    }
    else
    {
        appendSerialData(QString("Error: Could not create config.yaml in %1. Error: %2").arg(directoryPath, configFile.errorString()));
    }
}

void MainWindow::sendCommand(const QByteArray &command)
{
    // If a cycle recording is active, any command other than 'd' (display stall info)
    // should cancel the cycle to prevent saving incomplete data.
    if (isCycleRecordingActive && command != "d")
    {
        isCycleRecordingActive = false;
        recordedCycleData.clear();
        appendSerialData(QString("=== Auto-cycle recording CANCELLED by user command: %1 ===").arg(QString(command)));
    }

    if (serialPort && serialPort->isOpen() && serialPort->isWritable())
    {
        serialPort->write(command + "\n");
        appendSerialData(QString("=== Command Sent: %1 ===").arg(QString(command)));
    }
    else
    {
        appendSerialData("Error: Cannot send command. Serial port not connected or not writable.");
    }
}

void MainWindow::populateSerialPorts()
{
    ui->portComboBox->clear();
    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &port : ports)
    {
        ui->portComboBox->addItem(port.portName());
    }

    // Set default selection to COM10 if available
    int com10Index = ui->portComboBox->findText("COM10");
    if (com10Index != -1)
    {
        ui->portComboBox->setCurrentIndex(com10Index);
    }

    appendSerialData("=== Serial port list refreshed ===");
}

void MainWindow::populateFileList()
{
    ui->fileSelectionComboBox->clear();
    QString dirPath = ui->recordingPathLineEdit->text().trimmed();
    if (dirPath.isEmpty())
    {
        return; // Nothing to do if directory is empty
    }

    QDir dir(dirPath);
    if (!dir.exists())
    {
        return; // Nothing to list if directory doesn't exist
    }

    QStringList nameFilters;
    nameFilters << "*.csv";
    QFileInfoList fileList = dir.entryInfoList(nameFilters, QDir::Files);

    for (const QFileInfo &fileInfo : fileList)
    {
        // Add all CSVs except the metadata file itself
        if (fileInfo.fileName() != "signal_metadata.csv")
        {
            ui->fileSelectionComboBox->addItem(fileInfo.baseName());
        }
    }
}

void MainWindow::onSaveMetadataClicked()
{
    QString dirPath = ui->recordingPathLineEdit->text().trimmed();
    QString fileNameToUpdate = ui->fileSelectionComboBox->currentText();

    if (dirPath.isEmpty() || fileNameToUpdate.isEmpty())
    {
        appendSerialData("Error: Directory and file must be specified to save metadata.");
        QMessageBox::critical(this, "Error", "Recording directory and a selected file are required.");
        return;
    }

    QString metadataFilePath = dirPath + "/signal_metadata.csv";
    if (!QFile::exists(metadataFilePath))
    {
        appendSerialData("Error: metadata file not found in the specified directory.");
        QMessageBox::critical(this, "Error", "signal_metadata.csv not found. Save a recording first.");
        return;
    }

    // Read the entire metadata file
    QFile metadataFile(metadataFilePath);
    if (!metadataFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        appendSerialData("Error: Could not open metadata file for reading.");
        return;
    }

    QStringList lines;
    QTextStream in(&metadataFile);
    while (!in.atEnd())
    {
        lines.append(in.readLine());
    }
    metadataFile.close();

    bool entryFound = false;
    for (int i = 1; i < lines.size(); ++i) // Start from 1 to skip header
    {
        QStringList fields = lines[i].split(',');
        if (!fields.isEmpty() && fields.first() == fileNameToUpdate)
        {
            entryFound = true;
            fields[3] = ui->participantLineEdit->text(); // participant
            fields[4] = ui->sbpLineEdit->text();         // sbp
            fields[5] = ui->dbpLineEdit->text();         // dbp
            lines[i] = fields.join(',');
            break;
        }
    }

    if (!entryFound)
    {
        appendSerialData(QString("Error: Entry for '%1' not found in metadata.").arg(fileNameToUpdate));
        return;
    }

    // Write the modified content back to the file
    if (!metadataFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        appendSerialData("Error: Could not open metadata file for writing.");
        return;
    }

    QTextStream out(&metadataFile);
    for (const QString &line : lines)
    {
        out << line << "\n";
    }
    metadataFile.close();

    appendSerialData(QString("=== Metadata for '%1' updated successfully ===").arg(fileNameToUpdate));

    // Animate button color for feedback
    auto animation = new QVariantAnimation(this);
    animation->setDuration(1500);                // 1.5-second fade
    animation->setStartValue(QColor("#27ae60")); // Green
    animation->setEndValue(QColor("#f0f0f0"));   // Default button gray

    connect(animation, &QVariantAnimation::valueChanged, [this](const QVariant &value)
            {
        QColor color = value.value<QColor>();
        // Automatically set text color for readability
        QString textColor = (color.lightnessF() > 0.5) ? "black" : "white";
        ui->saveMetadataButton->setStyleSheet(
            QString("background-color: %1; color: %2;").arg(color.name(), textColor)
        ); });

    // Reset stylesheet when done to restore hover effects etc.
    connect(animation, &QVariantAnimation::finished, [this]()
            { ui->saveMetadataButton->setStyleSheet(""); });

    animation->start(QAbstractAnimation::DeleteWhenStopped);
}

void MainWindow::updateMetadataFile(const QString &fileName, const QString &directory, double durationSeconds)
{
    QString metadataFilePath = directory + "/signal_metadata.csv";
    bool fileExists = QFile::exists(metadataFilePath);

    QFile metadataFile(metadataFilePath);
    if (metadataFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        QTextStream out(&metadataFile);

        // Write header only if the file is new
        if (!fileExists)
        {
            out << "name,source_dir,duration,participant,sbp,dbp,hr,fs\n";
        }

        // Get data from UI fields
        QString sourceDir = QDir(directory).dirName();
        QString participant = ui->participantLineEdit->text();
        QString sbp = ui->sbpLineEdit->text();
        QString dbp = ui->dbpLineEdit->text();
        QString hr = "N/A";
        QString fs = "100"; // Based on 10ms sampling interval in Arduino

        // Write the new data row
        out << fileName << ","
            << sourceDir << ","
            << QString::number(durationSeconds, 'f', 2) << ","
            << participant << ","
            << sbp << ","
            << dbp << ","
            << hr << ","
            << fs << "\n";

        metadataFile.close();
        appendSerialData("=== Metadata updated successfully ===");
    }
    else
    {
        appendSerialData(QString("Error: Could not update metadata file: %1").arg(metadataFile.errorString()));
    }
}
