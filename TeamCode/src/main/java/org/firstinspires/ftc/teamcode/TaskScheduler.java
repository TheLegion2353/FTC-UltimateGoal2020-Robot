package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Function;

public class TaskScheduler {
	public interface Runnable {
		public void run();
	}
	ArrayList<ArrayList<Runnable>> tasks = null;
	int taskIndex = 0;
	public TaskScheduler() {
		tasks = new ArrayList<ArrayList<Runnable>>();
	}

	public void addTasks(ArrayList<Runnable> functions) {
		tasks.add(functions);
	}

	public void addTasks(Runnable... functions) {
		tasks.add(new ArrayList<Runnable>(Arrays.asList(functions)));
	}

	public boolean update() {
		for (int i = 0; i < taskIndex; i++) {
			for (Runnable task : tasks.get(i)) {
				task.run();
			}
		}
		return tasks.size() == taskIndex - 1;
	}
}