import rosbag
import argparse
import os
import matplotlib.pyplot as plt
# run this like: python3 [location_of_this_file] --bag [path_to_bag] --topics [topic1] [topic2] ... --out [output_file_destination]
def extract_deltas_from_topics(bag_path, topic_names, output_path = None):
    if not os.path.exists(bag_path):
        print("Bag file not found.")
        return

    topic_deltas = {}

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic in topic_names:
            timestamps = []
            for _, msg, _ in bag.read_messages(topics=[topic]):
                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    ts = msg.header.stamp.to_nsec()
                    timestamps.append(ts)
            deltas = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
            topic_deltas[topic] = deltas
    if output_path:
        with open(output_path, 'w') as f:
            for topic in topic_names:
                deltas = topic_deltas.get(topic, [])
                for i, delta in enumerate(deltas):
                    f.write(f"{topic},{i},{delta}")

    print(f"Saved deltas for {len(topic_names)} topics to {output_path}")

    # Overlay plot
    plt.figure(figsize=(12, 6))
    for topic, deltas in topic_deltas.items():
        plt.plot(deltas, label=topic)

    plt.xlabel('Frame Index')
    plt.ylabel('Delta (nanoseconds)')
    plt.title('Overlay of Timestamp Deltas Across Topics')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract and plot timestamp deltas from multiple ROS topics")
    parser.add_argument("--bag", required=True, help="Path to the ROS bag file")
    parser.add_argument("--topics", nargs='+', required=True, help="List of topic names to extract timestamps from")
    parser.add_argument("--output", required=False, help="Output file path for delta timestamps")
    args = parser.parse_args()
    if(args.output):
        extract_deltas_from_topics(args.bag, args.topics, args.output)
    else:
        extract_deltas_from_topics(args.bag, args.topics)