#!/usr/bin/env python3
"""Wait for specified ROS2 topics to have publishers."""
import sys
import subprocess

def check_topic_has_publisher(topic):
    """Check if topic has at least one publisher using ros2 topic info."""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'info', topic],
            capture_output=True, text=True, timeout=2
        )
        for line in result.stdout.split('\n'):
            if 'Publisher count:' in line:
                count = int(line.split(':')[1].strip())
                return count > 0
    except:
        pass
    return False

def main():
    if len(sys.argv) < 2:
        print('Usage: wait_for_topics.py <topic1> [topic2] ...')
        sys.exit(1)

    topics = set(sys.argv[1:])
    ready = set()

    print(f'Waiting for: {list(topics)}')

    import time
    while ready != topics:
        time.sleep(0.5)
        for topic in topics - ready:
            if check_topic_has_publisher(topic):
                ready.add(topic)
                print(f'Ready: {topic}')

    print('All topics ready!')
    sys.exit(0)

if __name__ == '__main__':
    main()
