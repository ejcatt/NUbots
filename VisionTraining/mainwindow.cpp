#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string>
#include <QFileDialog>
#include <QInputDialog>
#include "labeleditor.h"
#include "labelgenerator.h"
#include "visionoptimiser.h"
#include "visioncomparitor.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    QObject::connect(ui->autoPB, SIGNAL(clicked()), this, SLOT(generateLabels()));
    QObject::connect(ui->modPB, SIGNAL(clicked()), this, SLOT(modifyLabels()));
    QObject::connect(ui->compPB, SIGNAL(clicked()), this, SLOT(compareParams()));
    QObject::connect(ui->optPB, SIGNAL(clicked()), this, SLOT(runOptimiser()));
    QObject::connect(ui->exitPB, SIGNAL(clicked()), this, SLOT(close()));
    QObject::connect(ui->browsePB, SIGNAL(clicked()), this, SLOT(getDirectory()));
    QObject::connect(ui->gridPB, SIGNAL(clicked()), this, SLOT(gridSearch()));
    QObject::connect(ui->evalPB, SIGNAL(clicked()), this, SLOT(evaluate()));

    ui->dirEdit->setText(QString(getenv("HOME")) + QString("/nubot/"));
}

MainWindow::~MainWindow()
{
    delete ui;
}

/**
  * @brief Retrieves and stores a directory selection from the user.
  */
void MainWindow::getDirectory()
{
    QString dir = QFileDialog::getExistingDirectory(this, "Select Directory", (string(getenv("HOME")) + string("/Images/FYP/Final100/")).c_str());
    if(!dir.isNull())
        ui->dirEdit->setText(dir);
    setFocus();
}

/**
  * @brief Runs the auto label generation.
  */
void MainWindow::generateLabels()
{
    LabelGenerator lg(this);
    lg.show();
    lg.run(ui->dirEdit->text().toStdString());
    setFocus();
}

/**
  * @brief Runs the label modifier.
  */
void MainWindow::modifyLabels()
{
    //get the image file
    QString imagename = QFileDialog::getOpenFileName(this, "Select Image Stream", (string(getenv("HOME")) + string("/Images/FYP/Final100/")).c_str());
    if(!imagename.isNull()) {
        //get the label file
        QString labelname = QFileDialog::getOpenFileName(this, "Select Labels File", (string(getenv("HOME")) + string("/Images/FYP/Final100/")).c_str());
        if(!labelname.isNull()) {
            LabelEditor le(this);
            le.show();
            le.run(ui->dirEdit->text().toStdString(), labelname.toStdString(), imagename.toStdString());
            setFocus();
        }
    }
}

/**
  * @brief Runs the parameter comparison utility.
  */
void MainWindow::compareParams()
{
    //get the first parameter file
    QString config0 = QFileDialog::getOpenFileName(this, "Select First Config", (string(getenv("HOME")) + string("/Images/FYP/Final100/")).c_str());
    if(!config0.isNull()) {
        //get the second parameter file
        QString config1 = QFileDialog::getOpenFileName(this, "Select First Config", (string(getenv("HOME")) + string("/Images/FYP/Final100/")).c_str());
        if(!config1.isNull()) {
            string image = ui->dirEdit->text().toStdString() + string("image.strm");
            string lut = ui->dirEdit->text().toStdString() + string("default.lut");

            VisionComparitor comp;
            comp.show();
            comp.run(image, lut, config0.toStdString(), config1.toStdString());
        }
    }
    setFocus();
}

/**
  * @brief Runs the optimisation utility.
  */
void MainWindow::runOptimiser()
{
    bool ok;
    QStringList l;
    l.append("PSO");
    l.append("PGRL");
    l.append("EHCLS");
    l.append("PGA");
    //get the optimiser choice from the user
    QString s = QInputDialog::getItem(this, "Select Optimiser", "Select the preferred optimiser", l, 0, false, &ok);
    if(ok) {
        //get the number of iterations from the user
        int iterations = QInputDialog::getInt(this, "Iterations", "Select the number of optimiser iterations.", 100, 1, 1000000, 1, &ok);
        if(ok) {
            VisionOptimiser opt(this, VisionOptimiser::getChoiceFromString(s.toStdString()));
            opt.show();

            opt.run(ui->dirEdit->text().toStdString(), iterations);
        }
    }
    setFocus();
}

/**
  * @brief Runs the grid search utility.
  */
void MainWindow::gridSearch()
{
    VisionOptimiser opt;
    opt.gridSearch("/home/shannon/gridsearch/" , 20);
}

/**
  * @brief Runs the evaluation utility.
  */
void MainWindow::evaluate()
{
    VisionOptimiser opt;
    for(int i=0; i<100; i++)
        opt.errorPandRevaluation(ui->dirEdit->text().toStdString());
}
