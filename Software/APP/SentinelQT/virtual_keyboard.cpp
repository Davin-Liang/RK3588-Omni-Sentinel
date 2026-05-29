#include "virtual_keyboard.h"

#include <QGridLayout>
#include <QLineEdit>
#include <QPushButton>

VirtualKeyboard::VirtualKeyboard(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(120);
    setMaximumHeight(160);
    build_layout_();
    hide_keyboard();
}

void VirtualKeyboard::show_for(QLineEdit* target)
{
    if (!target) return;
    target_ = target;
    target_->setFocus();
    show();
    emit visibilityChanged(true);
}

void VirtualKeyboard::hide_keyboard()
{
    if (target_) {
        target_->clearFocus();
        target_ = nullptr;
    }
    hide();
    emit visibilityChanged(false);
}

// ============================================================================
// 布局
// ============================================================================

void VirtualKeyboard::build_layout_()
{
    QGridLayout* grid = new QGridLayout(this);
    grid->setSpacing(4);
    grid->setContentsMargins(4, 4, 4, 4);

    const int btnW = 50;
    const int btnH = 40;

    struct KeyRow {
        const char* keys[4];
    };
    KeyRow rows[] = {
        {{ "7",  "8",  "9",  "." }},
        {{ "4",  "5",  "6",  "-" }},
        {{ "1",  "2",  "3",  "\xe2\x86\x90" }},   // ←
        {{ "0",  "Del", "Done", "" }},
    };

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            QString key = QString::fromUtf8(rows[r].keys[c]);
            if (key.isEmpty()) continue;

            QPushButton* btn = make_button_(key, btnW, btnH);
            connect(btn, &QPushButton::clicked, this, [this, key]() {
                on_key_clicked_(key);
            });
            grid->addWidget(btn, r, c);
        }
    }
}

QPushButton* VirtualKeyboard::make_button_(const QString& text, int w, int h)
{
    QPushButton* btn = new QPushButton(text, this);
    btn->setMinimumSize(w, h);
    btn->setMaximumSize(w * 2, h);
    btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    btn->setStyleSheet(
        "QPushButton {"
        "  font-size: 16px; font-weight: 600;"
        "  color: #2d3535;"
        "  background-color: #F5F0D7;"
        "  border: 1px solid #8b949e;"
        "  border-radius: 6px;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #d4c9a8;"
        "}"
    );
    return btn;
}

// ============================================================================
// 按键处理
// ============================================================================

void VirtualKeyboard::on_key_clicked_(const QString& key)
{
    if (!target_) return;

    QString t = target_->text();
    int pos = target_->cursorPosition();

    if (key == "\xe2\x86\x90") {         // ← 退格
        if (pos > 0) {
            t.remove(pos - 1, 1);
            target_->setText(t);
            target_->setCursorPosition(pos - 1);
        }
    } else if (key == "Del") {
        if (pos < t.length()) {
            t.remove(pos, 1);
            target_->setText(t);
            target_->setCursorPosition(pos);
        }
    } else if (key == "Done") {
        hide_keyboard();
    } else {
        // 数字/小数点/负号
        t.insert(pos, key);
        target_->setText(t);
        target_->setCursorPosition(pos + key.length());
    }
}
