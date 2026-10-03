---
name: pi_connect
description: Подключение к Raspberry Pi 5 проекта MAZE2027 (madness@192.168.78.26) по SSH — выполнить команду, посмотреть логи, скопировать файлы, проверить лидар LD19. Использовать, когда пользователь просит что-то сделать «на распберри», «на пи», «на малине».
---

# Raspberry Pi (MAZE2027)

- Хост: `madness@192.168.78.26` (Pi 5, пользователь `madness`)
- Проект на Pi: `~/maze`, venv: `~/maze/venv`
- Лидар LD19: `/dev/ttyAMA3`
- `sudo` требует пароль — не запускать `sudo` через SSH, а отдать команду пользователю.

## 1. Проверка связи

Всегда начинать с неинтерактивной проверки (пароль ввести нельзя):

```bash
ssh -o BatchMode=yes -o ConnectTimeout=5 madness@192.168.78.26 'hostname && uptime'
```

- **Timed out / No route to host** — Pi выключена, не в той сети или IP сменился. Попросить пользователя проверить питание/Wi‑Fi и IP (`hostname -I` на самой Pi). Можно попробовать `madness@madeness.local` (hostname Pi — `madeness`), если mDNS работает.
- **Permission denied (publickey,password)** — ключ не установлен. Попросить пользователя один раз выполнить у себя в терминале (вводит пароль сам):

  ```bash
  ssh-copy-id -i ~/.ssh/id_ed25519.pub madness@192.168.78.26
  ```

- **Host key verification failed / REMOTE HOST IDENTIFICATION HAS CHANGED** — Pi переустанавливали. Сказать пользователю, предложить `ssh-keygen -R 192.168.78.26`; самому не удалять без согласия.

## 2. Выполнение команд

Одноразовые команды — через `ssh ... '<команда>'` с `BatchMode=yes`. Для Python — активировать venv:

```bash
ssh -o BatchMode=yes madness@192.168.78.26 'cd ~/maze && source venv/bin/activate && python script.py'
```

- Долгие процессы (чтение лидара, робот) — ограничивать `timeout 10 ...`, чтобы сессия не зависала, либо запускать через Bash с `run_in_background`.
- Не оставлять висящих процессов: после теста проверить `pgrep -af python` и остановить свои.
- Изменяющие систему действия (перезагрузка, `apt`, правка `/boot/firmware/config.txt`, удаление файлов) — только с подтверждения пользователя.

## 3. Копирование файлов

```bash
scp -o BatchMode=yes local_file.py madness@192.168.78.26:~/maze/
rsync -avz --exclude venv --exclude __pycache__ ./pi/ madness@192.168.78.26:~/maze/
```

Перед перезаписью файлов на Pi посмотреть, что там лежит (`ssh ... 'ls -la ~/maze'`).

## 4. Полезные проверки

- Порт лидара: `ls -l /dev/ttyAMA3 && groups` (пользователь должен быть в `dialout`)
- Сырые данные LD19 (230400 бод): `timeout 3 python -c "import serial; s=serial.Serial('/dev/ttyAMA3',230400,timeout=1); print(s.read(94).hex())"` (в venv)
- Температура/троттлинг: `vcgencmd measure_temp; vcgencmd get_throttled`

## Когда команду выполняет сам пользователь

Если нужен `sudo` или SSH недоступен из Claude — давать команды в «локальной» форме (без обёртки `ssh`), т.к. пользователь вставляет их в уже открытую SSH‑сессию на Pi. Одна команда — один блок `bash`.
