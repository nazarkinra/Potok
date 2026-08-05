import pygame
import sys

# Инициализация подсистемы джойстиков
pygame.init()
pygame.joystick.init()

# Проверка наличия подключенных геймпадов
if pygame.joystick.get_count() == 0:
    print("Геймпад не найден. Убедитесь, что геймпад Hoco подключен по USB или Bluetooth.")
    sys.exit()

# Подключение к первому найденному геймпаду
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Успешно подключен геймпад: {joystick.get_name()}")
print("Нажимайте кнопки или стики. Для выхода нажмите Ctrl+C в терминале.")

try:
    while True:
        # Обработка всех поступающих событий
        for event in pygame.event.get():
            
            # Нажатие обычной кнопки (A, B, X, Y, бамперы)
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Кнопка нажата: {event.button}")
            
            # Отпускание кнопки
            if event.type == pygame.JOYBUTTONUP:
                print(f"Кнопка отпущена: {event.button}")
            
            # Движение стиков и курков (оси)
            if event.type == pygame.JOYAXISMOTION:
                # Фильтруем микроколебания (мертвая зона), так как стики редко стоят ровно в нуле
                if abs(event.value) > 0.1:
                    print(f"Ось {event.axis} отклонена: {event.value:.2f}")

            # Нажатия крестовины (D-pad)
            if event.type == pygame.JOYHATMOTION:
                print(f"Крестовина (Hat) {event.hat} в положении: {event.value}")

except KeyboardInterrupt:
    print("\nЗавершение работы...")
    pygame.quit()
