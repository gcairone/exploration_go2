#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum
import time

from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class State(Enum):
    """Stati della macchina a stati del GoalManager."""
    GOAL_CHOICE = 1
    NAVIGATING = 2
    WAIT_FOR_UPDATE = 3
    ABORTING = 4


class GoalManager(Node):
    """
    Nodo ROS2 che gestisce l'orchestrazione dell'esplorazione basata su frontiera.
    Riceve candidati target dal FrontierScanner e li invia al bt_navigator.
    """
    
    def __init__(self):
        super().__init__('goal_manager')
        
        self.declare_parameter('dynamic', True)
        self.declare_parameter('action_server_name', 'bt_navigator')
        self.declare_parameter('change_threshold', 0.2)
        self.declare_parameter('candidates_topic', '/frontier_candidates')
        self.declare_parameter('scores_topic', '/frontier_scores')
        self.declare_parameter('frontiers_topic', '/frontiers')
        self.declare_parameter('thresh_front', 10)
        
        self.dynamic = self.get_parameter('dynamic').get_parameter_value().bool_value
        self.action_server_name = self.get_parameter('action_server_name').get_parameter_value().string_value
        self.change_threshold = self.get_parameter('change_threshold').get_parameter_value().double_value
        self.candidates_topic = self.get_parameter('candidates_topic').get_parameter_value().string_value
        self.scores_topic = self.get_parameter('scores_topic').get_parameter_value().string_value
        self.frontiers_topic = self.get_parameter('frontiers_topic').get_parameter_value().string_value
        self.thresh_front = self.get_parameter('thresh_front').get_parameter_value().integer_value
        
        self.state = State.GOAL_CHOICE
        self.candidates = []
        self.scores = []
        self.index = 0
        self.current_goal_handle = None
        self.current_score = None
        self.num_frontiers = 0
        
        self.start_time = time.time()
        self.goals_sent = 0
        
        self.candidates_sub = self.create_subscription(
            MarkerArray,
            self.candidates_topic,
            self.candidates_callback,
            10
        )
        
        self.scores_sub = self.create_subscription(
            Float32MultiArray,
            self.scores_topic,
            self.scores_callback,
            10
        )
        
        self.frontiers_sub = self.create_subscription(
            PointCloud2,
            self.frontiers_topic,
            self.frontiers_callback,
            10
        )
        
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.action_server_name
        )
        
        self.get_logger().info(f'GoalManager inizializzato con parametri:')
        self.get_logger().info(f'  dynamic: {self.dynamic}')
        self.get_logger().info(f'  action_server_name: {self.action_server_name}')
        self.get_logger().info(f'  change_threshold: {self.change_threshold}')
        self.get_logger().info(f'  candidates_topic: {self.candidates_topic}')
        self.get_logger().info(f'  scores_topic: {self.scores_topic}')
        self.get_logger().info(f'  frontiers_topic: {self.frontiers_topic}')
        self.get_logger().info(f'  thresh_front: {self.thresh_front}')
        
        self.get_logger().info(f'Attendo che l\'action server {self.action_server_name} sia disponibile...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn(f'Action server {self.action_server_name} non disponibile dopo 10 secondi')
        else:
            self.get_logger().info(f'Action server {self.action_server_name} disponibile')
    
    def frontiers_callback(self, msg):
        """Callback per ricevere il numero di frontiere rilevate."""
        point_step = msg.point_step
        if point_step > 0:
            self.num_frontiers = len(msg.data) // point_step
        else:
            self.num_frontiers = 0
        
        # self.get_logger().debug(f'Frontiere rilevate: {self.num_frontiers}')
    
    def candidates_callback(self, msg):
        """Callback per i candidati target ricevuti dal FrontierScanner."""
        self.get_logger().info(f'Ricevuti {len(msg.markers)} candidati')
        
        self.candidates = msg.markers
        self.index = 0  # Reset dell'indice
        
        #    return
        
        self.process_update()
    
    def scores_callback(self, msg):
        """Callback per i punteggi dei candidati."""
        self.get_logger().debug(f'Ricevuti {len(msg.data)} punteggi')
        self.scores = list(msg.data)
        
    
    def process_update(self):
        """Processa l'aggiornamento della classifica."""
        if not self.candidates or not self.scores:
            self.get_logger().debug('Nessun candidato o punteggio disponibile')
            return
        
        if len(self.candidates) != len(self.scores):
            self.get_logger().debug('Lunghezze diverse tra candidati e punteggi')
            return
        
        new_best_score = self.scores[0] if self.scores else 0.0
        #self.execute_goal_choice()
        
        if self.state == State.NAVIGATING and self.dynamic:
            if self.current_score is not None:
                if self.current_score > 0:
                    relative_improvement = (new_best_score - self.current_score) / self.current_score
                else:
                    relative_improvement = new_best_score if new_best_score > 0 else 0.0
                
                self.get_logger().info(f'Confronto punteggi - Corrente: {self.current_score:.3f}, '
                                     f'Nuovo migliore: {new_best_score:.3f}, '
                                     f'Miglioramento relativo: {relative_improvement:.3f}')
                
                if relative_improvement > self.change_threshold:
                    self.get_logger().info(f'Nuovo candidato significativamente migliore, '
                                         f'aborting goal corrente')
                    self.transition_to(State.ABORTING)
                    #self.abort_current_goal()


                    if self.current_goal_handle:
                        self.get_logger().info('Invio richiesta di cancellazione del goal corrente')
                        cancel_future = self.current_goal_handle.cancel_goal_async()
                        cancel_future.add_done_callback(self.cancel_done_callback)
                    else:
                        self.index = 0
                        self.transition_to(State.GOAL_CHOICE)
                        self.execute_goal_choice()

        
        elif self.state == State.WAIT_FOR_UPDATE:
            # Eravamo in attesa di un aggiornamento
            self.get_logger().info('Ricevuto aggiornamento in WAIT_FOR_UPDATE')
            self.transition_to(State.GOAL_CHOICE)
            self.execute_goal_choice()
        elif self.state == State.GOAL_CHOICE:
            self.get_logger().info('Ricevuto aggiornamento in GOAL_CHOICE, avvio selezione obiettivo')
            self.execute_goal_choice()
    
    def transition_to(self, new_state):
        """Transizione tra stati."""
        self.get_logger().info(f'Transizione: {self.state.name} -> {new_state.name}')
        self.state = new_state
    
    def execute_goal_choice(self):
        """Esegue la logica dello stato GOAL_CHOICE."""
        if self.state != State.GOAL_CHOICE:
            return
        
        # Verifica se l'i-esimo obiettivo esiste
        if self.index < len(self.candidates):
            self.send_goal()
        else:
            self.get_logger().warn(f'Obiettivo {self.index} non disponibile nella classifica di {len(self.candidates)} elementi')
            self.transition_to(State.WAIT_FOR_UPDATE)
            self.get_logger().info('In attesa del prossimo aggiornamento della classifica...')
    
    def send_goal(self):
        """Invia il goal corrente al navigator."""
        if self.index >= len(self.candidates):
            self.get_logger().error('Tentativo di inviare goal fuori range')
            return
        
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Action server non disponibile')
            return
        
        current_candidate = self.candidates[self.index]
        current_score = self.scores[self.index] if self.index < len(self.scores) else 0.0
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header = current_candidate.header
        goal_msg.pose.pose = current_candidate.pose
        
        self.get_logger().info(f'Invio goal {self.index + 1}/{len(self.candidates)} '
                             f'con punteggio {current_score:.3f}')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, current_score)
        )
        
        self.goals_sent += 1
    
    def goal_response_callback(self, future, score):
        """Callback per la risposta del goal inviato."""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Errore nell\'invio del goal: {e}')
            self.handle_goal_completion(False)
            return
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rifiutato dal server')
            self.handle_goal_completion(False)
            return
        
        self.current_goal_handle = goal_handle
        self.current_score = score
        self.transition_to(State.NAVIGATING)
        
        self.get_logger().info('Goal accettato, in esecuzione...')
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback per il risultato del goal."""
        self.current_goal_handle = None
        self.current_score = None
        
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Errore nel ricevere il risultato: {e}')
            self.handle_goal_completion(False)
            return
        
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal completato con successo')
            self.handle_goal_completion(True)
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal abortito')
            self.handle_goal_completion(False)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal cancellato (preemption dinamica)')
            self.index = 0  
            self.transition_to(State.GOAL_CHOICE)
            self.execute_goal_choice()
        else:
            self.get_logger().warn(f'Goal terminato con status: {status}')
            self.handle_goal_completion(False)
    
    def handle_goal_completion(self, success):
        """Gestisce il completamento di un goal (successo o fallimento)."""
        if self.state == State.ABORTING:
            self.index = 0  # Reset per prendere il nuovo migliore
            self.transition_to(State.GOAL_CHOICE)
            self.execute_goal_choice()
            return
        self.index += 1
        
        self.transition_to(State.GOAL_CHOICE)
        self.execute_goal_choice()
    
    def abort_current_goal(self):
        """Abortisce il goal corrente."""
        if self.current_goal_handle:
            self.get_logger().info('Invio richiesta di cancellazione del goal corrente')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.index = 0
            self.transition_to(State.GOAL_CHOICE)
            self.execute_goal_choice()
    
    def cancel_done_callback(self, future):
        """Callback per la cancellazione del goal."""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Goal cancellato con successo')
            else:
                self.get_logger().warn('Impossibile cancellare il goal')
        except Exception as e:
            self.get_logger().error(f'Errore nella cancellazione del goal: {e}')
    
    def check_exploration_finished(self):
        """Verifica se l'esplorazione è terminata."""
        if self.num_frontiers > self.thresh_front:
            elapsed_time = time.time() - self.start_time
            self.get_logger().error(f'Esplorazione FALLITA: {self.num_frontiers} frontiere presenti '
                                  f'ma nessun candidato disponibile')
            self.get_logger().info(f'Statistiche esplorazione:')
            self.get_logger().info(f'  Tempo totale: {elapsed_time:.2f} secondi')
            self.get_logger().info(f'  Obiettivi inviati: {self.goals_sent}')
            self.terminate_exploration(False)
        else:
            elapsed_time = time.time() - self.start_time
            self.get_logger().info(f'Esplorazione COMPLETATA CON SUCCESSO!')
            self.get_logger().info(f'Statistiche esplorazione:')
            self.get_logger().info(f'  Tempo totale: {elapsed_time:.2f} secondi')
            self.get_logger().info(f'  Obiettivi inviati: {self.goals_sent}')
            self.get_logger().info(f'  Frontiere rimanenti: {self.num_frontiers}')
            self.terminate_exploration(True)
    
    def terminate_exploration(self, success):
        """Termina l'esplorazione e ferma il nodo."""
        if success:
            self.get_logger().info('Terminazione nodo per completamento esplorazione')
        else:
            self.get_logger().error('Terminazione nodo per fallimento esplorazione')
        
        raise SystemExit
    
    def feedback_callback(self, feedback_msg):
        """Callback per il feedback del goal (opzionale)."""
        pass


def main(args=None):
    rclpy.init(args=args)
    
    goal_manager = GoalManager()
    
    try:
        rclpy.spin(goal_manager)
    except (KeyboardInterrupt, SystemExit):
        goal_manager.get_logger().info('Terminazione goal_manager')
    finally:
        goal_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()