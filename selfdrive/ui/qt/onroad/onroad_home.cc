#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QPainter>
#include <QStackedLayout>


#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>


#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/carrot.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QOpenGLWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  //main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setContentsMargins(UI_BORDER_SIZE, 0, UI_BORDER_SIZE, 0);

  QFont font;
  font.setPixelSize(27);
  font.setWeight(QFont::DemiBold);
  QHBoxLayout* topLayout = new QHBoxLayout();
  topLeftLabel = new QLabel("", this);
  topLeftLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  topLeftLabel->setAlignment(Qt::AlignLeft);
  topLeftLabel->setFont(font);
  topLeftLabel->setStyleSheet("QLabel { color : white; }");
  topLayout->addWidget(topLeftLabel);
  topLabel = new QLabel("", this);
  topLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  topLabel->setAlignment(Qt::AlignCenter);
  topLabel->setFont(font);
  topLabel->setStyleSheet("QLabel { color : white; }");
  topLayout->addWidget(topLabel);
  topRightLabel = new QLabel("", this);
  topRightLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  topRightLabel->setAlignment(Qt::AlignRight);
  topRightLabel->setFont(font);
  topRightLabel->setStyleSheet("QLabel { color : white; }");
  topLayout->addWidget(topRightLabel);
  main_layout->addLayout(topLayout);

  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QHBoxLayout* bottomLayout = new QHBoxLayout();
  bottomLeftLabel = new QLabel("", this);
  bottomLeftLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  bottomLeftLabel->setAlignment(Qt::AlignLeft);
  bottomLeftLabel->setFont(font);
  bottomLeftLabel->setStyleSheet("QLabel { color : white; }");
  bottomLayout->addWidget(bottomLeftLabel);
  bottomLabel = new QLabel("", this);
  bottomLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  bottomLabel->setAlignment(Qt::AlignCenter);
  bottomLabel->setFont(font);
  bottomLabel->setStyleSheet("QLabel { color : white; }");
  bottomLayout->addWidget(bottomLabel);
  bottomRightLabel = new QLabel("", this);
  bottomRightLabel->setFixedHeight(27); // 높이를 30 픽셀로 설정
  bottomRightLabel->setAlignment(Qt::AlignRight);
  bottomRightLabel->setFont(font);
  bottomRightLabel->setStyleSheet("QLabel { color : white; }");
  bottomLayout->addWidget(bottomRightLabel);
  main_layout->addLayout(bottomLayout);


  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, this);
    split->insertWidget(0, arCam);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  //alerts->updateState(s);
  ui_update_alert(OnroadAlerts::getAlert(*(s.sm), s.scene.started_frame));
  nvg->updateState(s);

  QColor bgColor = bg_colors[s.status];
  QColor bgColor_long = bg_colors[s.status];
  const SubMaster& sm = *(s.sm);
  const auto car_control = sm["carControl"].getCarControl();
  //if (s.status == STATUS_DISENGAGED && car_control.getLatActive()) {
  //    bgColor = bg_colors[STATUS_LAT_ACTIVE];
  //}
  const auto car_state = sm["carState"].getCarState();
  if (car_state.getSteeringPressed()) {
      bgColor = bg_colors[STATUS_OVERRIDE];
  }
  else if (car_control.getLatActive()) {
      bgColor = bg_colors[STATUS_ENGAGED];
  }
  else
      bgColor = bg_colors[STATUS_DISENGAGED];

  if (car_state.getGasPressed()) {
      bgColor_long = bg_colors[STATUS_OVERRIDE];
  }
  else if (car_control.getLongActive()) {
      bgColor_long = bg_colors[STATUS_ENGAGED];
  }
  else
      bgColor_long = bg_colors[STATUS_DISENGAGED];
  if (bg != bgColor || bg_long != bgColor_long) {
    // repaint border
    bg = bgColor;
    bg_long = bgColor_long;
    //update();
  }
  update();
}

void OnroadWindow::offroadTransition(bool offroad) {
  alerts->clear();
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    p.beginNativePainting();
    UIState* s = uiState();
    extern void ui_draw_border(UIState * s, int w, int h, QColor bg, QColor bg_long);
    ui_draw_border(s, width(), height(), bg, bg_long);
    p.endNativePainting();
}


// OnroadWindow.cpp에서 OpenGL 초기화 및 그리기 구현
void OnroadWindow::initializeGL() {
    initializeOpenGLFunctions(); // QOpenGLFunctions 초기화

    // Parent widget을 위한 NanoVG 컨텍스트 생성
    //s->vg = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
    //if (s->vg == nullptr) {
    //    printf("Could not init nanovg.\n");
    //    return;
    //}
}
