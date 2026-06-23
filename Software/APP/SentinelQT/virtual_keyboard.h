#ifndef VIRTUAL_KEYBOARD_H
#define VIRTUAL_KEYBOARD_H

#include <QWidget>

class QLineEdit;
class QPushButton;

class VirtualKeyboard : public QWidget {
    Q_OBJECT

public:
    explicit VirtualKeyboard(QWidget* parent = nullptr);

    void show_for(QLineEdit* target);
    void hide_keyboard();

signals:
    void visibilityChanged(bool visible);

private slots:
    void on_key_clicked_(const QString& key);

private:
    QPushButton* make_button_(const QString& text, int w, int h);
    void build_layout_();

    QLineEdit* target_ = nullptr;
};

#endif // VIRTUAL_KEYBOARD_H
